"""
tests/mock_arduino.py
=====================

Thread-safe Arduino simulator with the same public interface as
:class:`~core.serial_manager.SerialManager`.

Use this as a drop-in replacement wherever ``SerialManager`` is required so
that tests and UI development can run without a physical Arduino connected.

Public interface (mirrors SerialManager)
-----------------------------------------
* ``telem_queue``    — ``queue.Queue[TelemetryFrame]``
* ``response_queue`` — ``queue.Queue[str]``
* ``is_connected``   — ``bool`` property
* ``start()``        — begin background TELEM thread
* ``stop()``         — stop background thread
* ``send_command(cmd)`` — parse and respond to an Arduino command

Supported commands
------------------
All commands from the wire protocol defined in ``command_interface.py`` are
handled:

    CMD HOME
    CMD STOP
    CMD ESTOP
    CMD PAUSE
    CMD RESUME
    CMD GET_STATE
    CMD JOG <UP|DOWN> <speed> [accel]
    CMD MOVE <dist> [speed] [accel]
    CMD RUN_PROFILE <dip_spd> <wdraw_spd> <accel> <depth> <dwell_bot_ms> <dwell_top_ms> <n_dips>
    CMD SET_TELEM_RATE <hz>
    CMD SET_SOFT_LIMITS <min_mm> <max_mm>
    CMD BEGIN_SEGMENTED_MOVE <n_segs>
    CMD MOVE_SEG  <dist> <speed> [accel]
    CMD DWELL_SEG <duration_ms>
    CMD RUN_LOADED_MOVE

State machine
-------------
The mock enforces the same valid-state guards as the real Arduino firmware:

    IDLE    → can HOME
    HOMING  → (transitions to READY automatically)
    READY   → can HOME, RUN_PROFILE, BEGIN_SEGMENTED_MOVE, JOG, MOVE, STOP, ESTOP
    RUNNING → can PAUSE, STOP, ESTOP  (transitions to READY when run completes)
    PAUSED  → can RESUME, STOP, ESTOP
    ERROR   → can HOME, ESTOP

Speed multiplier
----------------
Pass ``speed_multiplier`` to accelerate simulated motion for fast tests.
``speed_multiplier=10`` means a real 4-second descent completes in 0.4 s.
Default is ``10.0``.

Usage example::

    from tests.mock_arduino import MockArduino
    from core.command_interface import CommandInterface

    mock = MockArduino(speed_multiplier=20.0)
    mock.start()

    ci = CommandInterface(mock)
    ci.home()                          # ACK HOME, then HOMING → READY
    ci.run(profile)                    # simulate full run

    mock.stop()
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Final, Optional

from core.telemetry_parser import TelemetryFrame

# ---------------------------------------------------------------------------
# Internal motion-plan types
# ---------------------------------------------------------------------------

@dataclass
class _MovePlan:
    """One linear-motion phase: position interpolates from start to end."""
    start_pos:  float
    end_pos:    float
    start_t:    float   # monotonic time when this phase begins (set by _plan_motion)
    duration_s: float
    speed_mm_s: float   # magnitude, used for vel_commanded_mm_s
    state:      str
    phase:      str


@dataclass
class _DwellPlan:
    """One hold phase: position stays constant for a fixed duration."""
    pos:        float
    start_t:    float   # set by _plan_motion
    duration_s: float
    state:      str
    phase:      str


_MotionStep = _MovePlan | _DwellPlan

# ---------------------------------------------------------------------------
# State-guard sets (which Arduino states allow each command)
# ---------------------------------------------------------------------------

_HOME_STATES:   Final[frozenset[str]] = frozenset({"IDLE", "READY", "ERROR"})
_STOP_STATES:   Final[frozenset[str]] = frozenset({"READY", "RUNNING", "PAUSED"})
_PAUSE_STATES:  Final[frozenset[str]] = frozenset({"RUNNING"})
_RESUME_STATES: Final[frozenset[str]] = frozenset({"PAUSED"})
_RUN_STATES:    Final[frozenset[str]] = frozenset({"READY"})
_JOG_STATES:    Final[frozenset[str]] = frozenset({"READY"})

# How long the mock takes to simulate homing (before multiplier is applied).
_HOMING_REAL_S: Final[float] = 0.5


# ---------------------------------------------------------------------------
# MockArduino
# ---------------------------------------------------------------------------

class MockArduino:
    """
    Thread-safe Arduino simulator.

    Exposes the same public interface as
    :class:`~core.serial_manager.SerialManager` so it can be injected
    directly into :class:`~core.command_interface.CommandInterface`.

    Parameters
    ----------
    speed_multiplier:
        Factor by which all simulated motion durations are divided.
        ``10.0`` makes a real 4-second descent complete in 0.4 s.
        Set to ``1.0`` for real-time simulation.
    telem_hz_default:
        Initial telemetry broadcast rate.  Can be changed at runtime via
        ``CMD SET_TELEM_RATE``.
    """

    def __init__(
        self,
        speed_multiplier:  float = 10.0,
        telem_hz_default:  int   = 10,
    ) -> None:
        self._speed_mult: float = max(speed_multiplier, 0.01)

        # ---- Public queues (same interface as SerialManager) ----------------
        self.telem_queue:    queue.Queue[TelemetryFrame] = queue.Queue(maxsize=1000)
        self.response_queue: queue.Queue[str]            = queue.Queue(maxsize=200)
        # raw_queue receives a copy of every TX command and RX line as plain
        # strings ("TX CMD HOME", "RX ACK HOME", "RX TELEM,...") for the
        # serial monitor tab.
        self.raw_queue:      queue.Queue[str]            = queue.Queue(maxsize=2000)

        # ---- Internal state -------------------------------------------------
        self._lock:  threading.Lock = threading.Lock()

        self._state: str   = "IDLE"
        self._phase: str   = "NONE"
        self._pos_mm: float = 0.0
        self._vel_commanded_mm_s: float = 0.0

        self._soft_min_mm: float = -900.0
        self._soft_max_mm: float =  900.0

        # ---- Telemetry rate -------------------------------------------------
        self._telem_hz:     int   = telem_hz_default
        self._t_start:      float = 0.0   # set in start()

        # ---- Motion plan ----------------------------------------------------
        self._motion_plan: list[_MotionStep] = []
        self._plan_idx:    int   = 0
        self._next_state:  str   = "READY"
        self._next_phase:  str   = "NONE"
        self._final_pos:   float = 0.0

        # ---- Segment collection state ---------------------------------------
        self._collecting:         bool        = False
        self._n_segs_expected:    int         = 0
        self._n_segs_received:    int         = 0
        self._segments_loaded:    bool        = False
        self._collected_segs:     list[dict]  = []

        # ---- Background thread ----------------------------------------------
        self._running: bool = False
        self._thread:  Optional[threading.Thread] = None

    # -------------------------------------------------------------------------
    # SerialManager public interface
    # -------------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """``True`` while the background thread is running."""
        return self._running

    def start(self) -> None:
        """Start the background telemetry and state-transition thread."""
        self._t_start = time.monotonic()
        self._running = True
        self._thread  = threading.Thread(
            target=self._background_loop,
            daemon=True,
            name="mock-arduino",
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the background thread and clean up."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send_command(self, cmd: str) -> None:
        """
        Parse *cmd* and place the appropriate response on ``response_queue``.

        Intermediate ``MOVE_SEG`` / ``DWELL_SEG`` lines during segment
        collection do not produce a response until the last segment is
        received, at which point ``ACK PROFILE_READY`` is queued.

        Args:
            cmd: Raw command string, e.g. ``"CMD HOME\\n"``.
        """
        cmd    = cmd.strip()
        # Log every outgoing command to raw_queue for the serial monitor.
        try:
            self.raw_queue.put_nowait(f"TX {cmd}")
        except queue.Full:
            pass

        tokens = cmd.split()
        if not tokens or tokens[0] != "CMD":
            return

        verb: str       = tokens[1] if len(tokens) > 1 else ""
        args: list[str] = tokens[2:]

        with self._lock:
            response: Optional[str] = self._dispatch(verb, args)

        if response is not None:
            self.response_queue.put(response)
            # Mirror response to raw_queue.
            try:
                self.raw_queue.put_nowait(f"RX {response}")
            except queue.Full:
                pass

    # -------------------------------------------------------------------------
    # Command dispatch
    # -------------------------------------------------------------------------

    def _dispatch(self, verb: str, args: list[str]) -> Optional[str]:
        """Route *verb* to the appropriate handler.  Called with lock held."""

        # During segment collection, only MOVE_SEG and DWELL_SEG are valid.
        if self._collecting:
            if verb == "MOVE_SEG":
                return self._handle_move_seg(args)
            if verb == "DWELL_SEG":
                return self._handle_dwell_seg(args)
            return f"ERR {verb} unexpected command during segment collection"

        dispatch: dict = {
            "HOME":                 lambda: self._handle_home(),
            "STOP":                 lambda: self._handle_stop(),
            "ESTOP":                lambda: self._handle_estop(),
            "PAUSE":                lambda: self._handle_pause(),
            "RESUME":               lambda: self._handle_resume(),
            "GET_STATE":            lambda: self._handle_get_state(),
            "JOG":                  lambda: self._handle_jog(args),
            "MOVE":                 lambda: self._handle_move(args),
            "RUN_PROFILE":          lambda: self._handle_run_profile(args),
            "SET_TELEM_RATE":       lambda: self._handle_set_telem_rate(args),
            "SET_SOFT_LIMITS":      lambda: self._handle_set_soft_limits(args),
            "BEGIN_SEGMENTED_MOVE": lambda: self._handle_begin_segmented_move(args),
            "RUN_LOADED_MOVE":      lambda: self._handle_run_loaded_move(),
        }

        handler = dispatch.get(verb)
        if handler is None:
            return f"ERR {verb} unknown command"

        try:
            return handler()
        except (ValueError, IndexError) as exc:
            return f"ERR {verb} {exc}"

    # -------------------------------------------------------------------------
    # State guard helper
    # -------------------------------------------------------------------------

    def _check_state(self, verb: str, valid: frozenset[str]) -> Optional[str]:
        """Return an ERR string if *self._state* is not in *valid*, else None."""
        if self._state not in valid:
            return f"ERR {verb} invalid state {self._state}"
        return None

    # -------------------------------------------------------------------------
    # Command handlers (called with lock held)
    # -------------------------------------------------------------------------

    def _handle_home(self) -> str:
        err = self._check_state("HOME", _HOME_STATES)
        if err:
            return err
        self._state = "HOMING"
        self._phase = "NONE"
        self._vel_commanded_mm_s = 0.0
        dur = _HOMING_REAL_S / self._speed_mult
        self._plan_motion(
            [_DwellPlan(pos=self._pos_mm, start_t=0.0, duration_s=dur,
                        state="HOMING", phase="NONE")],
            next_state="READY", next_phase="NONE", final_pos=0.0,
        )
        return "ACK HOME"

    def _handle_stop(self) -> str:
        err = self._check_state("STOP", _STOP_STATES)
        if err:
            return err
        self._state = "READY"
        self._phase = "NONE"
        self._vel_commanded_mm_s = 0.0
        self._motion_plan = []
        self._plan_idx    = 0
        return "ACK STOP"

    def _handle_estop(self) -> str:
        self._state = "ERROR"
        self._phase = "NONE"
        self._vel_commanded_mm_s = 0.0
        self._motion_plan = []
        self._plan_idx    = 0
        return "ACK ESTOP"

    def _handle_pause(self) -> str:
        err = self._check_state("PAUSE", _PAUSE_STATES)
        if err:
            return err
        self._state = "PAUSED"
        self._vel_commanded_mm_s = 0.0
        return "ACK PAUSE"

    def _handle_resume(self) -> str:
        err = self._check_state("RESUME", _RESUME_STATES)
        if err:
            return err
        self._state = "RUNNING"
        # Re-anchor remaining plan steps to the current time so timing continues.
        now = time.monotonic()
        if self._plan_idx < len(self._motion_plan):
            shift = now - self._motion_plan[self._plan_idx].start_t
            for step in self._motion_plan[self._plan_idx:]:
                step.start_t += shift
        return "ACK RESUME"

    def _handle_get_state(self) -> str:
        return f"STATE {self._state} {self._phase}"

    def _handle_jog(self, args: list[str]) -> str:
        err = self._check_state("JOG", _JOG_STATES)
        if err:
            return err
        if len(args) < 2:
            return "ERR JOG missing arguments"
        direction = args[0]
        speed_mm_s = float(args[1])
        if direction not in ("UP", "DOWN"):
            return f"ERR JOG invalid direction {direction!r}"
        sign = 1.0 if direction == "UP" else -1.0
        self._state = "RUNNING"
        self._phase = "NONE"
        self._vel_commanded_mm_s = sign * speed_mm_s
        return "ACK JOG"

    def _handle_move(self, args: list[str]) -> str:
        err = self._check_state("MOVE", _RUN_STATES)
        if err:
            return err
        if not args:
            return "ERR MOVE missing distance"
        dist_mm    = float(args[0])
        speed_mm_s = float(args[1]) if len(args) > 1 else 10.0
        dur        = abs(dist_mm) / max(speed_mm_s, 0.01) / self._speed_mult
        start      = self._pos_mm
        self._state = "RUNNING"
        self._plan_motion(
            [_MovePlan(start_pos=start, end_pos=start + dist_mm,
                       start_t=0.0, duration_s=dur,
                       speed_mm_s=speed_mm_s, state="RUNNING", phase="NONE")],
            next_state="READY", next_phase="NONE", final_pos=start + dist_mm,
        )
        return "ACK MOVE"

    def _handle_run_profile(self, args: list[str]) -> str:
        err = self._check_state("RUN_PROFILE", _RUN_STATES)
        if err:
            return err
        if len(args) < 7:
            return "ERR RUN_PROFILE missing arguments"

        dip_spd      = float(args[0])
        wdraw_spd    = float(args[1])
        # accel      = float(args[2])  # not used in simplified linear simulation
        depth_mm     = float(args[3])
        dwell_bot_ms = int(float(args[4]))
        dwell_top_ms = int(float(args[5]))
        n_dips       = int(float(args[6]))

        plans: list[_MotionStep] = []
        pos = self._pos_mm

        for dip_i in range(n_dips):
            bot = pos - depth_mm
            plans.append(_MovePlan(
                start_pos=pos, end_pos=bot,
                start_t=0.0,
                duration_s=depth_mm / max(dip_spd, 0.01) / self._speed_mult,
                speed_mm_s=dip_spd, state="RUNNING", phase="DESCENDING",
            ))
            if dwell_bot_ms > 0:
                plans.append(_DwellPlan(
                    pos=bot, start_t=0.0,
                    duration_s=dwell_bot_ms / 1000.0 / self._speed_mult,
                    state="RUNNING", phase="DWELL_BOTTOM",
                ))
            plans.append(_MovePlan(
                start_pos=bot, end_pos=pos,
                start_t=0.0,
                duration_s=depth_mm / max(wdraw_spd, 0.01) / self._speed_mult,
                speed_mm_s=wdraw_spd, state="RUNNING", phase="ASCENDING",
            ))
            # DWELL_TOP between dips (not after the last dip)
            if dwell_top_ms > 0 and dip_i < n_dips - 1:
                plans.append(_DwellPlan(
                    pos=pos, start_t=0.0,
                    duration_s=dwell_top_ms / 1000.0 / self._speed_mult,
                    state="RUNNING", phase="DWELL_TOP",
                ))

        self._state = "RUNNING"
        self._plan_motion(plans, next_state="READY", next_phase="NONE", final_pos=pos)
        return "ACK RUN_PROFILE"

    def _handle_set_telem_rate(self, args: list[str]) -> str:
        if not args:
            return "ERR SET_TELEM_RATE missing hz"
        hz = int(float(args[0]))
        if hz < 0 or hz > 50:
            return f"ERR SET_TELEM_RATE invalid hz {hz}"
        self._telem_hz = hz
        return "ACK SET_TELEM_RATE"

    def _handle_set_soft_limits(self, args: list[str]) -> str:
        if len(args) < 2:
            return "ERR SET_SOFT_LIMITS missing arguments"
        min_mm = float(args[0])
        max_mm = float(args[1])
        if min_mm >= max_mm:
            return "ERR SET_SOFT_LIMITS min must be less than max"
        self._soft_min_mm = min_mm
        self._soft_max_mm = max_mm
        return "ACK SET_SOFT_LIMITS"

    def _handle_begin_segmented_move(self, args: list[str]) -> str:
        err = self._check_state("BEGIN_SEGMENTED_MOVE", _RUN_STATES)
        if err:
            return err
        if not args:
            return "ERR BEGIN_SEGMENTED_MOVE missing n_segs"
        n = int(float(args[0]))
        if n <= 0 or n > 64:
            return f"ERR BEGIN_SEGMENTED_MOVE invalid n_segs {n}"
        self._collecting      = True
        self._n_segs_expected = n
        self._n_segs_received = 0
        self._segments_loaded = False
        self._collected_segs  = []
        return "ACK BEGIN_SEGMENTED_MOVE"

    def _handle_move_seg(self, args: list[str]) -> Optional[str]:
        if len(args) < 2:
            self._collecting = False
            return "ERR MOVE_SEG missing arguments"
        dist_mm    = float(args[0])
        speed_mm_s = float(args[1])
        accel      = float(args[2]) if len(args) > 2 else 30.0
        self._collected_segs.append({
            "type": "move",
            "distance_mm": dist_mm,
            "speed_mm_s":  speed_mm_s,
            "accel_mm_s2": accel,
        })
        self._n_segs_received += 1
        if self._n_segs_received >= self._n_segs_expected:
            self._collecting      = False
            self._segments_loaded = True
            return "ACK PROFILE_READY"
        return None   # no ACK for intermediate segments

    def _handle_dwell_seg(self, args: list[str]) -> Optional[str]:
        if not args:
            self._collecting = False
            return "ERR DWELL_SEG missing duration"
        ms = int(float(args[0]))
        self._collected_segs.append({"type": "dwell", "duration_ms": ms})
        self._n_segs_received += 1
        if self._n_segs_received >= self._n_segs_expected:
            self._collecting      = False
            self._segments_loaded = True
            return "ACK PROFILE_READY"
        return None

    def _handle_run_loaded_move(self) -> str:
        if self._state != "READY":
            return f"ERR RUN_LOADED_MOVE invalid state {self._state}"
        if not self._segments_loaded:
            return "ERR RUN_LOADED_MOVE no segments loaded"

        plans: list[_MotionStep] = []
        pos = self._pos_mm
        for seg in self._collected_segs:
            if seg["type"] == "move":
                dist_mm    = float(seg["distance_mm"])
                speed_mm_s = float(seg["speed_mm_s"])
                dur        = abs(dist_mm) / max(speed_mm_s, 0.01) / self._speed_mult
                plans.append(_MovePlan(
                    start_pos=pos, end_pos=pos + dist_mm,
                    start_t=0.0, duration_s=dur,
                    speed_mm_s=speed_mm_s, state="RUNNING", phase="NONE",
                ))
                pos += dist_mm
            elif seg["type"] == "dwell":
                ms  = float(seg["duration_ms"])
                dur = ms / 1000.0 / self._speed_mult
                plans.append(_DwellPlan(
                    pos=pos, start_t=0.0, duration_s=dur,
                    state="RUNNING", phase="NONE",
                ))

        self._state           = "RUNNING"
        self._segments_loaded = False
        self._plan_motion(plans, next_state="READY", next_phase="NONE", final_pos=pos)
        return "ACK RUN_LOADED_MOVE"

    # -------------------------------------------------------------------------
    # Motion plan
    # -------------------------------------------------------------------------

    def _plan_motion(
        self,
        plans:      list[_MotionStep],
        next_state: str,
        next_phase: str,
        final_pos:  float,
    ) -> None:
        """Set the motion plan and stamp each step with a sequential start time."""
        now = time.monotonic()
        t   = now
        for step in plans:
            step.start_t  = t
            t            += step.duration_s

        self._motion_plan = plans
        self._plan_idx    = 0
        self._next_state  = next_state
        self._next_phase  = next_phase
        self._final_pos   = final_pos

    # -------------------------------------------------------------------------
    # Background loop
    # -------------------------------------------------------------------------

    def _background_loop(self) -> None:
        """Advance the motion plan and broadcast TELEM at the configured rate."""
        next_telem_t: float = time.monotonic()

        while self._running:
            now = time.monotonic()

            with self._lock:
                self._advance_motion_plan(now)

                frame: Optional[TelemetryFrame] = None
                if self._telem_hz > 0 and now >= next_telem_t:
                    frame        = self._make_frame(now)
                    next_telem_t = now + 1.0 / self._telem_hz

            if frame is not None:
                try:
                    self.telem_queue.put_nowait(frame)
                except queue.Full:
                    pass   # drop on full — same behaviour as real firmware
                # Mirror raw TELEM string to raw_queue for the serial monitor.
                raw_telem = (
                    f"RX TELEM,{frame.timestamp_ms},"
                    f"{frame.pos_mm:.3f},{frame.vel_actual_mm_s:.2f},"
                    f"{frame.vel_commanded_mm_s:.2f},{frame.accel_mm_s2:.2f},"
                    f"{frame.state},{frame.phase}"
                )
                try:
                    self.raw_queue.put_nowait(raw_telem)
                except queue.Full:
                    pass

            time.sleep(0.01)   # 100 Hz internal poll

    def _advance_motion_plan(self, now: float) -> None:
        """Update position / state / velocity from the current plan step."""
        if not self._motion_plan or self._plan_idx >= len(self._motion_plan):
            return

        # Paused: freeze position in place, do not advance plan.
        if self._state == "PAUSED":
            return

        step    = self._motion_plan[self._plan_idx]
        elapsed = now - step.start_t

        if isinstance(step, _MovePlan):
            if step.duration_s > 0:
                frac = min(elapsed / step.duration_s, 1.0)
                self._pos_mm = (
                    step.start_pos + frac * (step.end_pos - step.start_pos)
                )
            else:
                self._pos_mm = step.end_pos
            sign = 1.0 if step.end_pos >= step.start_pos else -1.0
            self._vel_commanded_mm_s = sign * step.speed_mm_s
        else:
            # _DwellPlan
            self._pos_mm             = step.pos
            self._vel_commanded_mm_s = 0.0

        self._state = step.state
        self._phase = step.phase

        if elapsed >= step.duration_s:
            self._plan_idx += 1
            if self._plan_idx >= len(self._motion_plan):
                # All steps complete — transition to final state.
                self._state              = self._next_state
                self._phase              = self._next_phase
                self._pos_mm             = self._final_pos
                self._vel_commanded_mm_s = 0.0
                self._motion_plan        = []
                self._plan_idx           = 0

    def _make_frame(self, now: float) -> TelemetryFrame:
        """Build a ``TelemetryFrame`` from the current mock state."""
        ms = int((now - self._t_start) * 1000)
        return TelemetryFrame(
            timestamp_ms       = ms,
            pos_mm             = round(self._pos_mm, 3),
            vel_actual_mm_s    = self._vel_commanded_mm_s,
            vel_commanded_mm_s = self._vel_commanded_mm_s,
            accel_mm_s2        = 0.0,
            state              = self._state,
            phase              = self._phase,
        )

    # -------------------------------------------------------------------------
    # Test helpers
    # -------------------------------------------------------------------------

    def inject_error(
        self,
        error_code: str = "ENDSTOP_TRIGGERED_UNEXPECTEDLY",
    ) -> None:
        """Force the mock into ``ERROR`` state — simulates an unexpected fault.

        Args:
            error_code: Error label placed in the next TELEM frame's phase
                        field for diagnostic purposes.
        """
        with self._lock:
            self._state              = "ERROR"
            self._phase              = "NONE"
            self._vel_commanded_mm_s = 0.0
            self._motion_plan        = []
            self._plan_idx           = 0

    def set_position(self, pos_mm: float) -> None:
        """Teleport the simulated carriage to *pos_mm* — useful for test setup.

        Does not send any TELEM; the new position appears on the next
        scheduled frame.
        """
        with self._lock:
            self._pos_mm = pos_mm
