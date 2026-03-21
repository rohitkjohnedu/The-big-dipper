"""
core/serial_manager.py

Owns the serial.Serial connection to the Arduino and runs the read loop
in a background thread.

Threading model (from the design doc):
  - Serial thread  : reads lines, routes TELEM to telem_queue,
                     ACK/ERR/STATE to response_queue.  Never touches the UI.
  - UI / main thread: calls send_command() which is thread-safe.

Usage:
    mgr = SerialManager(port="COM3", baud=115200)
    mgr.start()                          # opens port, starts read thread
    mgr.send_command("CMD HOME")
    response = mgr.response_queue.get(timeout=5)
    frame    = mgr.telem_queue.get(timeout=2)
    mgr.stop()
"""

import logging
import queue
import threading
import time

import serial

from core.telemetry_parser import TelemetryFrame, parse as parse_telem

log = logging.getLogger(__name__)


class SerialManager:
    """
    Manages a single serial connection to the Arduino.

    Attributes:
        telem_queue:    queue.Queue[TelemetryFrame]  — parsed telemetry frames.
        response_queue: queue.Queue[str]             — raw ACK/ERR/STATE lines.
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        """
        Args:
            port:    Serial port name ("COM3", "/dev/ttyUSB0", etc.).
            baud:    Baud rate — must match SERIAL_BAUD_RATE in config.h (115200).
            timeout: Read timeout in seconds for the serial port.
        """
        self._port    = port
        self._baud    = baud
        self._timeout = timeout

        self._serial:  serial.Serial | None = None
        self._thread:  threading.Thread | None = None
        self._running  = False
        self._lock     = threading.Lock()   # guards _serial.write()

        self.telem_queue:    queue.Queue[TelemetryFrame] = queue.Queue()
        self.response_queue: queue.Queue[str]            = queue.Queue()

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------

    def start(self) -> None:
        """Open the serial port and start the background read thread."""
        self._serial = serial.Serial(
            port     = self._port,
            baudrate = self._baud,
            timeout  = self._timeout,
        )
        # Give the Arduino time to reset after the serial port opens.
        # Without this delay the first command may arrive before the firmware
        # has finished its setup() routine.
        time.sleep(2.0)

        self._running = True
        self._thread  = threading.Thread(target=self._read_loop, daemon=True, name="serial-reader")
        self._thread.start()
        log.info("SerialManager started on %s at %d baud", self._port, self._baud)

    def stop(self) -> None:
        """Signal the read thread to stop and close the serial port."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
        log.info("SerialManager stopped")

    def send_command(self, cmd: str) -> None:
        """
        Send a command string to the Arduino.

        Thread-safe — may be called from any thread.
        A newline is appended automatically if not already present.

        Args:
            cmd: Command string, e.g. "CMD HOME" or "CMD JOG UP 10".
        """
        if not cmd.endswith("\n"):
            cmd += "\n"
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.write(cmd.encode("ascii"))
                log.debug("TX: %r", cmd.rstrip())
            else:
                log.warning("send_command called but serial port is not open")

    @property
    def is_connected(self) -> bool:
        """True if the serial port is open."""
        return bool(self._serial and self._serial.is_open)

    # -------------------------------------------------------------------------
    # Background read loop
    # -------------------------------------------------------------------------

    def _read_loop(self) -> None:
        """
        Read lines from the serial port and route them to the correct queue.

        Runs in the serial thread — never touches the UI.
        Exits when _running is set to False or the port is closed.
        """
        while self._running:
            try:
                raw = self._serial.readline()
            except serial.SerialException as exc:
                log.error("Serial read error: %s", exc)
                break

            if not raw:
                # readline() timed out — no data, loop and try again.
                continue

            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue

            log.debug("RX: %r", line)

            # Route TELEM lines to the telemetry queue, everything else
            # (ACK, ERR, STATE, freeform prints) to the response queue.
            if line.startswith("TELEM,"):
                frame = parse_telem(line)
                if frame is not None:
                    self.telem_queue.put(frame)
            else:
                self.response_queue.put(line)
