"""
core/serial_manager.py
======================

Owns the ``serial.Serial`` connection to the Arduino and runs the serial
read loop in a dedicated background thread.

Threading model
---------------
The design doc mandates three threads:

* **Serial thread** (this module): reads incoming lines from the Arduino,
  classifies each line as TELEM or a command response, and puts it onto the
  appropriate ``queue.Queue``.  This thread *never* touches PyQt6 widgets.
* **Data / UI thread**: consumes ``telem_queue`` and ``response_queue`` via
  Qt signals/slots.  Always runs in the main thread.

``send_command()`` is thread-safe and may be called from any thread.

Line routing
------------
Every line received from the Arduino is routed to one of two queues:

* Lines starting with ``"TELEM,"`` are parsed into ``TelemetryFrame`` objects
  and placed on ``telem_queue``.
* All other lines (``ACK``, ``ERR``, ``STATE``, freeform debug prints) are
  placed as raw strings on ``response_queue``.

Usage example::

    mgr = SerialManager(port="COM3", baud=115200)
    mgr.start()

    mgr.send_command("CMD HOME")
    ack = mgr.response_queue.get(timeout=5)   # blocks until ACK/ERR arrives

    frame = mgr.telem_queue.get(timeout=2)    # blocks until next TELEM frame

    mgr.stop()
"""

import logging
import queue
import threading
import time
from typing import Final, Optional

import serial

from core.telemetry_parser import TelemetryFrame
from core.telemetry_parser import parse as _parse_telem

# Module-level logger — messages appear under "core.serial_manager".
log: Final[logging.Logger] = logging.getLogger(__name__)

# Time (seconds) to wait after opening the serial port before sending any
# commands.  The Arduino resets when the port is opened; without this delay
# the first command may arrive before setup() has finished executing.
_ARDUINO_RESET_DELAY_S: Final[float] = 2.0


class SerialManager:
    """
    Manages a single serial connection to the Arduino.

    Opens the port in ``start()``, spawns a daemon read thread, and closes
    everything cleanly in ``stop()``.  The two public queues expose parsed
    data to the rest of the application without requiring any locking on the
    consumer side (``queue.Queue`` is already thread-safe).

    Attributes:
        telem_queue:    ``queue.Queue[TelemetryFrame]`` — parsed telemetry
                        frames, produced by the serial thread and consumed
                        by the data/UI thread.
        response_queue: ``queue.Queue[str]`` — raw ACK/ERR/STATE lines,
                        consumed by ``CommandInterface`` after each command.
    """

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        """
        Initialise the manager.  Does *not* open the serial port — call
        ``start()`` to do that.

        Args:
            port:    Serial port identifier, e.g. ``"COM3"`` on Windows or
                     ``"/dev/ttyUSB0"`` on Linux.
            baud:    Baud rate.  Must match ``SERIAL_BAUD_RATE`` in the
                     Arduino ``config.h`` (default 115200).
            timeout: Read timeout for ``serial.Serial`` in seconds.  Controls
                     how long ``readline()`` blocks before returning an empty
                     bytes object.  Shorter values make ``stop()`` faster to
                     return; longer values reduce CPU spinning.
        """
        # --- Connection parameters -------------------------------------------
        self._port:    str   = port
        self._baud:    int   = baud
        self._timeout: float = timeout

        # --- Internal state --------------------------------------------------
        # _serial is None until start() is called.
        self._serial:  Optional[serial.Serial] = None
        # _thread is None until start() is called.
        self._thread:  Optional[threading.Thread] = None
        # Flag used to signal the read thread to exit its loop.
        self._running: bool = False
        # Lock that serialises concurrent write() calls from different threads.
        self._write_lock: threading.Lock = threading.Lock()

        # --- Public queues ---------------------------------------------------
        # Bounded to prevent unbounded memory growth if the consumer (UI)
        # falls behind.  1000 telemetry frames at 50 Hz = 20 s of backlog
        # before frames are dropped — more than enough for any UI lag.
        # response_queue is capped at 200: commands are always ACK'd promptly
        # so a large backlog here indicates a protocol error.
        self.telem_queue:    queue.Queue[TelemetryFrame] = queue.Queue(maxsize=1000)
        self.response_queue: queue.Queue[str]            = queue.Queue(maxsize=200)
        # raw_queue receives a copy of every TX command and RX line as plain
        # strings ("TX CMD HOME", "RX ACK HOME", "RX TELEM,...") for the
        # serial monitor tab.
        self.raw_queue:      queue.Queue[str]            = queue.Queue(maxsize=2000)

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def start(self) -> None:
        """
        Open the serial port and start the background read thread.

        Blocks for ``_ARDUINO_RESET_DELAY_S`` seconds after opening the port
        to allow the Arduino to complete its reset cycle before any commands
        are sent.

        Raises:
            serial.SerialException: If the port cannot be opened (e.g. wrong
                                    port name, device not connected).
        """
        # Open the serial port.  The Arduino resets on DTR toggle (default
        # behaviour of pyserial) so we must wait before sending commands.
        self._serial = serial.Serial(
            port     = self._port,
            baudrate = self._baud,
            timeout  = self._timeout,
        )
        log.info("Serial port %s opened at %d baud", self._port, self._baud)

        # Wait for the Arduino to finish its setup() routine.
        log.debug("Waiting %.1f s for Arduino reset…", _ARDUINO_RESET_DELAY_S)
        time.sleep(_ARDUINO_RESET_DELAY_S)

        # Start the read thread as a daemon so it does not prevent the process
        # from exiting if stop() is not called explicitly (e.g. on crash).
        self._running = True
        self._thread  = threading.Thread(
            target = self._read_loop,
            daemon = True,
            name   = "serial-reader",
        )
        self._thread.start()
        log.info("SerialManager started on %s", self._port)

    def stop(self) -> None:
        """
        Signal the read thread to exit and close the serial port.

        Blocks until the read thread terminates (up to 3 seconds) before
        closing the port to avoid a race between the thread's last
        ``readline()`` call and the port being closed underneath it.
        """
        # Signal the thread to stop on its next loop iteration.
        self._running = False

        # Join the thread with a timeout so stop() does not hang indefinitely
        # if the thread is stuck in a long readline() call.
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                # Thread did not exit within the timeout — log a warning.
                # This can happen if readline() is blocked on a slow serial
                # port.  The daemon flag ensures it will not prevent process exit.
                log.warning(
                    "serial read thread did not exit within 3 s — continuing anyway"
                )
            self._thread = None

        # Close the serial port after the thread has exited.
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
            self._serial = None

        log.info("SerialManager stopped")

    # -------------------------------------------------------------------------
    # Public command interface
    # -------------------------------------------------------------------------

    def send_command(self, cmd: str) -> None:
        """
        Send a command string to the Arduino over the serial port.

        Thread-safe: protected by an internal write lock so this method may be
        called from the UI thread, the data thread, or any other context
        without risk of interleaved writes corrupting the serial stream.

        A newline character (``\\n``) is appended automatically if the string
        does not already end with one, matching the Arduino's ``\\n``-terminated
        line protocol.

        Args:
            cmd: Command string to send, e.g. ``"CMD HOME"`` or
                 ``"CMD JOG UP 10.0000"``.  Must be ASCII-encodable.

        Raises:
            RuntimeError: If the serial port is not open (``start()`` not
                called, or ``stop()`` already called).  Callers must not
                silently swallow this — a dropped command is always a bug.
        """
        # Ensure the line is newline-terminated before encoding.
        if not cmd.endswith("\n"):
            cmd += "\n"

        # Log every outgoing command to raw_queue for the serial monitor.
        try:
            self.raw_queue.put_nowait(f"TX {cmd.strip()}")
        except queue.Full:
            pass

        with self._write_lock:
            if self._serial is not None and self._serial.is_open:
                try:
                    self._serial.write(cmd.encode("ascii"))
                    log.debug("TX → %r", cmd.rstrip())
                except serial.SerialException as exc:
                    # Port was disconnected between the is_open check and write().
                    # Log and swallow — the read loop will detect the disconnect
                    # on its next readline() call and exit cleanly.
                    log.error("send_command(%r) failed: %s", cmd.rstrip(), exc)
            else:
                raise RuntimeError(
                    f"send_command({cmd.rstrip()!r}) failed — serial port is not open. "
                    "Call start() before sending commands."
                )

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """
        ``True`` if the serial port is currently open, ``False`` otherwise.

        Safe to call from any thread.
        """
        return self._serial is not None and self._serial.is_open

    # -------------------------------------------------------------------------
    # Background read loop (runs in serial thread)
    # -------------------------------------------------------------------------

    def _read_loop(self) -> None:
        """
        Read lines from the serial port and route them to the correct queue.

        This method runs in the serial thread for the lifetime of the
        connection.  It exits when ``_running`` is set to ``False`` (by
        ``stop()``) or when a ``SerialException`` occurs (e.g. USB disconnect).

        Line routing:

        * Lines starting with ``"TELEM,"`` are parsed into
          ``TelemetryFrame`` objects via ``telemetry_parser.parse()`` and
          placed on ``telem_queue``.  Malformed TELEM lines are silently
          dropped (the parser logs a warning).
        * All other lines are placed as raw strings on ``response_queue``
          for ``CommandInterface`` to consume.

        This method must *never* touch PyQt6 widgets — all UI updates must
        go through signals/slots on the main thread.
        """
        # start() guarantees _serial is not None before launching this thread,
        # but guard defensively so the thread exits cleanly rather than crashing.
        if self._serial is None:
            log.error("_read_loop entered with no serial port — exiting immediately")
            return

        while self._running:
            # --- Read one line from the serial port --------------------------
            try:
                raw_bytes: bytes = self._serial.readline()
            except serial.SerialException as exc:
                # Port disconnected or other hardware error — exit the loop.
                log.error("Serial read error: %s", exc)
                break

            # readline() returns b"" on timeout — no data available, loop again.
            if not raw_bytes:
                continue

            # Decode bytes to string, replacing any non-ASCII bytes with '?'
            # so a single corrupt byte does not crash the read loop.
            line: str = raw_bytes.decode("ascii", errors="replace").strip()

            if not line:
                # Skip blank lines (e.g. a lone \r\n from the Arduino).
                continue

            log.debug("RX ← %r", line)

            # Mirror every received line to raw_queue for the serial monitor.
            try:
                self.raw_queue.put_nowait(f"RX {line}")
            except queue.Full:
                pass

            # --- Route to the appropriate queue ------------------------------
            if line.startswith("TELEM,"):
                # Parse and enqueue — malformed lines return None and are dropped.
                frame: Optional[TelemetryFrame] = _parse_telem(line)
                if frame is not None:
                    self.telem_queue.put(frame)
            else:
                # ACK, ERR, STATE, or any freeform print from the firmware.
                self.response_queue.put(line)
