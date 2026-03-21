"""
core/command_interface.py
=========================

High-level, typed command interface between Python and the Arduino firmware.

Responsibilities
----------------
* Build correctly formatted command strings from typed Python arguments.
* Send each command via :class:`~core.serial_manager.SerialManager` and block
  until the Arduino responds with ``ACK`` or ``ERR``.
* Inspect :attr:`~core.profile.DipProfile.velocity_profile_type` and dispatch
  ``run()`` to the appropriate Arduino command sequence automatically — callers
  never need to know the wire protocol.

Command/response contract (from arduino ``command_parser.cpp``)
---------------------------------------------------------------
Every ``CMD ...`` sent to the Arduino produces exactly one response line:

* ``ACK <cmd>``  — command accepted and started.
* ``ERR <cmd> <reason>`` — command rejected.

The sole exception is the segmented-move protocol, where intermediate
``CMD MOVE_SEG`` / ``CMD DWELL_SEG`` lines produce no per-segment ACK — only
the *final* segment triggers ``ACK PROFILE_READY``.  :meth:`_run_segmented`
streams all segments first and awaits ``PROFILE_READY`` once at the end.

Wire formats (all values in mm, mm/s, mm/s², ms)
-------------------------------------------------
::

    CMD HOME
    CMD STOP
    CMD ESTOP
    CMD PAUSE
    CMD RESUME
    CMD GET_STATE                               → STATE <state> <phase>
    CMD JOG <UP|DOWN> <speed> [accel]
    CMD MOVE <dist> [speed] [accel]
    CMD RUN_PROFILE <dip_spd> <wdraw_spd> <accel> <depth> <dwell_bot_ms> <dwell_top_ms> <n_dips>
    CMD BEGIN_SEGMENTED_MOVE <n_segs>
    CMD MOVE_SEG  <dist> <speed> [accel]       (n_segs times, no per-seg ACK)
    CMD DWELL_SEG <duration_ms>                (counts toward n_segs)
    CMD RUN_LOADED_MOVE                        (after ACK PROFILE_READY)
    CMD SET_TELEM_RATE <hz>
    CMD SET_SOFT_LIMITS <min_mm> <max_mm>

Thread safety
-------------
All public methods are designed to be called from the UI (main) thread.  They
block briefly while awaiting the ACK, which is acceptable for a desktop GUI.
:meth:`estop` is the only method that does *not* wait for an ACK.
"""

import logging
import queue
from typing import Any, Final

from core.profile import DipProfile
from core.serial_manager import SerialManager

# Module-level logger — messages appear under "core.command_interface".
log: Final[logging.Logger] = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

# Maximum number of segments the Arduino firmware can buffer before execution.
# Mirrors MOVE_SEG_BUFFER_SIZE in arduino/dip_coater_firmware/config.h.
MOVE_SEG_BUFFER_SIZE: Final[int] = 64

# Default seconds to wait for an ACK before raising CommandError.
DEFAULT_ACK_TIMEOUT_S: Final[float] = 5.0

# Jog direction strings accepted by the Arduino (case-sensitive).
_VALID_JOG_DIRECTIONS: Final[frozenset[str]] = frozenset({"UP", "DOWN"})


# ---------------------------------------------------------------------------
# Custom exception
# ---------------------------------------------------------------------------

class CommandError(RuntimeError):
    """
    Raised when a command fails at the protocol level.

    Two conditions trigger this exception:

    * The Arduino replies with ``ERR <cmd> <reason>`` — the command was
      rejected in the current machine state or had invalid parameters.
    * No ``ACK`` arrives within ``ack_timeout_s`` seconds — the Arduino is
      unresponsive, the port was disconnected, or a previous command consumed
      the ACK that was meant for this one.

    The exception message always includes the expected ACK token or the raw
    ERR line received, making it easy to display in the UI status bar.
    """


# ---------------------------------------------------------------------------
# CommandInterface class
# ---------------------------------------------------------------------------

class CommandInterface:
    """
    Typed, ACK-aware interface for all Arduino commands.

    Wraps :class:`~core.serial_manager.SerialManager` to provide:

    * **String building** — callers pass typed Python values; this class
      formats them into correctly structured command strings.
    * **ACK verification** — every command (except :meth:`estop`) blocks
      until ``ACK`` or ``ERR`` arrives on the response queue.
    * **Dispatch** — :meth:`run` inspects
      :attr:`~core.profile.DipProfile.velocity_profile_type` and calls the
      correct private helper, keeping the UI completely ignorant of the
      wire protocol details.

    Usage example::

        mgr = SerialManager(port="COM3")
        mgr.start()

        ci = CommandInterface(mgr)
        ci.home()
        ci.run(profile)
        ci.stop()

        mgr.stop()

    Attributes:
        _mgr:           The :class:`~core.serial_manager.SerialManager` that
                        owns the serial port and response queue.
        _ack_timeout_s: Seconds to wait for an ACK before raising
                        :class:`CommandError`.
    """

    def __init__(
        self,
        manager: SerialManager,
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
    ) -> None:
        """
        Initialise the interface.

        Does not open the serial port — call ``manager.start()`` before
        issuing any commands.

        Args:
            manager:       A :class:`~core.serial_manager.SerialManager`
                           instance.  Must be started before any command
                           method is called.
            ack_timeout_s: Seconds to block waiting for an ACK response.
                           Increase this value if the Arduino is running a
                           slow homing sequence or if the USB connection has
                           high latency.  Defaults to
                           :data:`DEFAULT_ACK_TIMEOUT_S` (5.0 s).
        """
        self._mgr:           SerialManager = manager
        self._ack_timeout_s: float         = ack_timeout_s

    # -------------------------------------------------------------------------
    # Zero-argument commands
    # -------------------------------------------------------------------------

    def home(self) -> None:
        """
        Send ``CMD HOME`` and await ``ACK HOME``.

        Initiates the homing sequence: the Arduino drives the carriage toward
        the endstop and zeroes the position counter on contact.

        Valid states: ``IDLE``, ``READY``, ``ERROR``.

        Raises:
            CommandError: If the Arduino replies with ERR (e.g. wrong state)
                          or if no ACK arrives within the timeout.
        """
        self._mgr.send_command("CMD HOME")
        self._wait_ack("HOME")

    def stop(self) -> None:
        """
        Send ``CMD STOP`` (graceful stop) and await ``ACK STOP``.

        Decelerates to rest using the configured acceleration ramp.  The
        Arduino transitions to ``READY`` after the motor halts.

        Valid states: ``RUNNING``, ``PAUSED``, ``READY`` (to cancel a jog).

        Raises:
            CommandError: On ERR or timeout.
        """
        self._mgr.send_command("CMD STOP")
        self._wait_ack("STOP")

    def estop(self) -> None:
        """
        Send ``CMD ESTOP`` immediately without waiting for an ACK.

        Emergency stop cuts motor power as fast as possible.  Valid from
        *any* state — no state check is performed on the Arduino side.

        This method deliberately does **not** call :meth:`_wait_ack` so the
        command reaches the Arduino with minimum latency.  The ``ACK ESTOP``
        will still arrive on the response queue and will be discarded by the
        next :meth:`_wait_ack` call (logged as an unexpected response).

        Note:
            The E-stop button widget should call this method directly.  The
            UI must also visually reflect the stopped state immediately,
            without waiting for a telemetry frame to confirm.
        """
        self._mgr.send_command("CMD ESTOP")
        log.warning("ESTOP sent")

    def pause(self) -> None:
        """
        Send ``CMD PAUSE`` and await ``ACK PAUSE``.

        Decelerates the motor and holds position.  Resume with
        :meth:`resume`.

        Valid states: ``RUNNING``.

        Raises:
            CommandError: On ERR or timeout.
        """
        self._mgr.send_command("CMD PAUSE")
        self._wait_ack("PAUSE")

    def resume(self) -> None:
        """
        Send ``CMD RESUME`` and await ``ACK RESUME``.

        Restarts the interrupted move from the current position.

        Valid states: ``PAUSED``.

        Raises:
            CommandError: On ERR or timeout.
        """
        self._mgr.send_command("CMD RESUME")
        self._wait_ack("RESUME")

    # -------------------------------------------------------------------------
    # Commands that return data
    # -------------------------------------------------------------------------

    def get_state(self) -> tuple[str, str]:
        """
        Send ``CMD GET_STATE`` and return the current machine state and phase.

        The Arduino replies with a single ``STATE <state> <phase>`` line on
        the response queue.

        Returns:
            A ``(state, phase)`` tuple.  Both strings match the enumeration
            values defined in :mod:`~core.telemetry_parser`
            (e.g. ``("RUNNING", "DESCENDING")``).

        Raises:
            CommandError: If no response arrives within the timeout, or if
                          the response cannot be parsed as
                          ``STATE <state> <phase>``.
        """
        self._mgr.send_command("CMD GET_STATE")

        # Read the next response line — expected to be "STATE <state> <phase>".
        raw: str
        try:
            raw = self._mgr.response_queue.get(timeout=self._ack_timeout_s)
        except queue.Empty:
            raise CommandError(
                f"Timeout ({self._ack_timeout_s:.1f} s) waiting for STATE response"
            ) from None

        parts: list[str] = raw.split()
        if len(parts) != 3 or parts[0] != "STATE":
            raise CommandError(f"Unexpected GET_STATE response: {raw!r}")

        state: str = parts[1]
        phase: str = parts[2]
        return (state, phase)

    # -------------------------------------------------------------------------
    # Motion commands with arguments
    # -------------------------------------------------------------------------

    def jog(
        self,
        direction: str,
        speed_mm_s: float,
        accel_mm_s2: float = 0.0,
    ) -> None:
        """
        Send ``CMD JOG <direction> <speed> [accel]`` and await ``ACK JOG``.

        Starts a continuous jog in the specified direction.  The motor runs
        until :meth:`stop` or :meth:`estop` is called.

        Args:
            direction:   ``"UP"`` or ``"DOWN"`` (case-sensitive).
            speed_mm_s:  Jog speed in mm/s.  Must be > 0.
            accel_mm_s2: Acceleration ramp rate in mm/s².  If ≤ 0 the
                         argument is omitted and the Arduino uses its
                         configured default.

        Raises:
            ValueError:   If ``direction`` is not ``"UP"`` or ``"DOWN"``.
            CommandError: On ERR or timeout.
        """
        if direction not in _VALID_JOG_DIRECTIONS:
            raise ValueError(
                f"jog direction must be one of {sorted(_VALID_JOG_DIRECTIONS)}, "
                f"got {direction!r}"
            )

        # Build command string — omit accel when not supplied (Arduino default).
        cmd: str
        if accel_mm_s2 > 0:
            cmd = f"CMD JOG {direction} {speed_mm_s:.4f} {accel_mm_s2:.4f}"
        else:
            cmd = f"CMD JOG {direction} {speed_mm_s:.4f}"

        self._mgr.send_command(cmd)
        self._wait_ack("JOG")

    def run(self, profile: DipProfile) -> None:
        """
        Execute a dip-coating run, choosing the correct command sequence
        automatically based on ``profile.velocity_profile_type``.

        Dispatch table:

        +-----------------+-----------------------------------------------+
        | Profile type    | Commands sent                                 |
        +=================+===============================================+
        | ``trapezoidal`` | Single ``CMD RUN_PROFILE …``                  |
        +-----------------+-----------------------------------------------+
        | ``segmented``   | ``CMD BEGIN_SEGMENTED_MOVE`` → segments →     |
        |                 | ``CMD RUN_LOADED_MOVE``                       |
        +-----------------+-----------------------------------------------+
        | ``spline``      | Raises ``NotImplementedError``                |
        +-----------------+-----------------------------------------------+

        Args:
            profile: A validated :class:`~core.profile.DipProfile` instance.

        Raises:
            NotImplementedError: If ``velocity_profile_type`` is ``"spline"``.
            ValueError:          If the type is unknown, or if a segmented
                                 profile has 0 segments or more than
                                 :data:`MOVE_SEG_BUFFER_SIZE`.
            CommandError:        If any Arduino command returns ERR or times out.
        """
        vtype: str = profile.velocity_profile_type

        if vtype == "trapezoidal":
            self._run_trapezoidal(profile)
        elif vtype == "segmented":
            self._run_segmented(profile)
        elif vtype == "spline":
            raise NotImplementedError(
                "Spline velocity profile execution is not yet implemented"
            )
        else:
            raise ValueError(f"Unknown velocity_profile_type: {vtype!r}")

    def set_telem_rate(self, hz: int) -> None:
        """
        Set the Arduino telemetry broadcast rate and await ``ACK SET_TELEM_RATE``.

        Args:
            hz: Broadcast frequency in Hz.  ``0`` disables telemetry.
                The Arduino firmware enforces a maximum of 50 Hz.

        Raises:
            CommandError: On ERR or timeout.
        """
        self._mgr.send_command(f"CMD SET_TELEM_RATE {hz}")
        self._wait_ack("SET_TELEM_RATE")

    def set_soft_limits(self, min_mm: float, max_mm: float) -> None:
        """
        Set soft travel limits and await ``ACK SET_SOFT_LIMITS``.

        The Arduino enforces ``min_mm < max_mm`` and rejects the command if
        the constraint is violated.

        Args:
            min_mm: Lower travel limit in mm.  Must be less than ``max_mm``.
            max_mm: Upper travel limit in mm.

        Raises:
            CommandError: On ERR (e.g. ``min >= max``) or timeout.
        """
        self._mgr.send_command(
            f"CMD SET_SOFT_LIMITS {min_mm:.4f} {max_mm:.4f}"
        )
        self._wait_ack("SET_SOFT_LIMITS")

    # -------------------------------------------------------------------------
    # Private run helpers
    # -------------------------------------------------------------------------

    def _run_trapezoidal(self, profile: DipProfile) -> None:
        """
        Send a single ``CMD RUN_PROFILE`` for a trapezoidal profile.

        Wire format::

            CMD RUN_PROFILE <dip_spd> <wdraw_spd> <accel> <depth_mm>
                            <dwell_bot_ms> <dwell_top_ms> <n_dips>

        All float values are formatted to 4 decimal places.  Integer values
        (dwell times, dip count) are sent without a decimal point.

        Args:
            profile: A ``DipProfile`` with ``velocity_profile_type == "trapezoidal"``.

        Raises:
            CommandError: On ERR or timeout.
        """
        cmd: str = (
            f"CMD RUN_PROFILE"
            f" {profile.dip_speed_mm_s:.4f}"
            f" {profile.withdraw_speed_mm_s:.4f}"
            f" {profile.accel_mm_s2:.4f}"
            f" {profile.dip_depth_mm:.4f}"
            f" {profile.dwell_bottom_ms}"
            f" {profile.dwell_top_ms}"
            f" {profile.n_dips}"
        )
        self._mgr.send_command(cmd)
        self._wait_ack("RUN_PROFILE")

    def _run_segmented(self, profile: DipProfile) -> None:
        """
        Stream a segmented profile to the Arduino and trigger execution.

        Full protocol sequence::

            → CMD BEGIN_SEGMENTED_MOVE <n_segs>
            ← ACK BEGIN_SEGMENTED_MOVE
            → CMD MOVE_SEG  <dist_mm> <speed_mm_s> <accel_mm_s2>   (move step)
            → CMD DWELL_SEG <duration_ms>                           (dwell step)
            ← ACK PROFILE_READY                          (Arduino sends after last seg)
            → CMD RUN_LOADED_MOVE
            ← ACK RUN_LOADED_MOVE

        Intermediate ``MOVE_SEG`` / ``DWELL_SEG`` lines do **not** produce
        per-segment ACKs — the Arduino accumulates segments silently and only
        responds after the final one.  All segments are sent before
        ``PROFILE_READY`` is awaited.

        Args:
            profile: A ``DipProfile`` with ``velocity_profile_type == "segmented"``.

        Raises:
            ValueError:   If ``segments`` is empty or exceeds
                          :data:`MOVE_SEG_BUFFER_SIZE`.
            CommandError: On ERR or timeout at any protocol step.
        """
        # Extract the flat segment list from velocity_profile_data.
        segments: list[dict[str, Any]] = list(
            profile.velocity_profile_data.get("segments", [])
        )
        n_segs: int = len(segments)

        # Guard against empty or oversized lists before touching the serial port.
        if n_segs == 0:
            raise ValueError("Segmented profile contains no segments")
        if n_segs > MOVE_SEG_BUFFER_SIZE:
            raise ValueError(
                f"Segment count {n_segs} exceeds Arduino buffer limit "
                f"MOVE_SEG_BUFFER_SIZE ({MOVE_SEG_BUFFER_SIZE})"
            )

        # --- Step 1: announce segment count, enter Arduino collection mode ----
        self._mgr.send_command(f"CMD BEGIN_SEGMENTED_MOVE {n_segs}")
        self._wait_ack("BEGIN_SEGMENTED_MOVE")

        # --- Step 2: stream every segment ------------------------------------
        # The Arduino counts received segments internally; no ACK per segment.
        seg: dict[str, Any]
        for seg in segments:
            seg_type: str = str(seg.get("type", ""))

            if seg_type == "move":
                dist_mm:     float = float(seg["distance_mm"])
                speed_mm_s:  float = float(seg["speed_mm_s"])
                accel_mm_s2: float = float(seg.get("accel_mm_s2", 0.0))

                # Include accel only when explicitly provided — Arduino uses its
                # default when the argument is absent.
                if accel_mm_s2 > 0:
                    self._mgr.send_command(
                        f"CMD MOVE_SEG {dist_mm:.4f} {speed_mm_s:.4f} {accel_mm_s2:.4f}"
                    )
                else:
                    self._mgr.send_command(
                        f"CMD MOVE_SEG {dist_mm:.4f} {speed_mm_s:.4f}"
                    )

            elif seg_type == "dwell":
                duration_ms: int = int(seg["duration_ms"])
                self._mgr.send_command(f"CMD DWELL_SEG {duration_ms}")

            else:
                # The profile validator should have caught this already, but
                # guard here in case velocity_profile_data was mutated after
                # construction.
                raise ValueError(
                    f"Unknown segment type {seg_type!r} at index "
                    f"{segments.index(seg)}"
                )

        # --- Step 3: wait for PROFILE_READY (sent after the final segment) ---
        self._wait_ack("PROFILE_READY")

        # --- Step 4: trigger execution of the loaded segment buffer ----------
        self._mgr.send_command("CMD RUN_LOADED_MOVE")
        self._wait_ack("RUN_LOADED_MOVE")

    # -------------------------------------------------------------------------
    # Private ACK/ERR helper
    # -------------------------------------------------------------------------

    def _wait_ack(
        self,
        expected: str,
        timeout_s: float | None = None,
    ) -> None:
        """
        Block until the Arduino acknowledges a command or reports an error.

        Reads lines from :attr:`~core.serial_manager.SerialManager.response_queue`
        until one of three conditions is met:

        1. A line matching ``"ACK <expected>"`` is received → return normally.
        2. A line starting with ``"ERR"`` is received → raise
           :class:`CommandError` with the raw ERR line.
        3. The queue is empty for ``timeout_s`` seconds → raise
           :class:`CommandError` with a timeout message.

        Lines that match neither ``ACK`` nor ``ERR`` are logged at DEBUG level
        and discarded.  This handles stale ``STATE`` responses or telemetry
        lines that slipped into the response queue.

        Args:
            expected:  The command token expected after ``"ACK "``
                       (e.g. ``"HOME"``, ``"PROFILE_READY"``).
            timeout_s: Override the instance timeout for this call.
                       Defaults to ``self._ack_timeout_s``.

        Raises:
            CommandError: On ERR response or queue-empty timeout.
        """
        effective_timeout: float = (
            timeout_s if timeout_s is not None else self._ack_timeout_s
        )
        ack_line: str = f"ACK {expected}"

        try:
            while True:
                # Block for up to effective_timeout seconds.
                raw: str = self._mgr.response_queue.get(timeout=effective_timeout)

                if raw == ack_line:
                    # Expected ACK received — command accepted by Arduino.
                    log.debug("← %s", raw)
                    return

                if raw.startswith("ERR"):
                    # Arduino rejected the command.
                    log.error("← %s (expected %s)", raw, ack_line)
                    raise CommandError(f"Arduino returned error: {raw!r}")

                # Unexpected line (stale STATE, debug print, etc.) — discard.
                log.debug("_wait_ack: discarding unexpected response %r", raw)

        except queue.Empty:
            raise CommandError(
                f"Timeout ({effective_timeout:.1f} s) waiting for {ack_line!r}"
            ) from None
