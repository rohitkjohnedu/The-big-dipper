"""
tests/hardware/test_hardware.py
================================

Hardware-in-the-loop tests that require a physical Arduino connected via
serial.  All tests in this module are marked ``@pytest.mark.hardware`` and
are **skipped automatically** when the port cannot be opened.

Running
-------
Run only hardware tests::

    uv run pytest -m hardware --hw-port COM4 -v

Run everything (hardware tests skip gracefully if no Arduino)::

    uv run pytest --hw-port COM4 -v

Test order
----------
Tests are designed to run in declaration order.  pytest does not guarantee
order by default, but hardware tests have natural dependencies:

    connection → telemetry → commands → home → trapezoidal run → segmented run

If a test earlier in the sequence fails (e.g. home fails), later tests that
depend on READY state will still run but may also fail or produce unexpected
results.  This is intentional — the failure chain makes root cause obvious.
"""

from __future__ import annotations

import queue
import time
from typing import Final

import pytest

import numpy as np

from core.command_interface import CommandInterface, CommandError
from core.data_recorder import DataRecorder, Float64Array, RecordedRun
from core.profile import DipProfile
from core.serial_manager import SerialManager
from core.telemetry_parser import TelemetryFrame
from motion.parabolic_profile import ParabolicProfile
from motion.spline_profile import SplineProfile
from motion.trapezoidal_profile import TrapezoidalProfile
from motion.velocity_profile import MoveSegment
from tests.conftest import (
    hw_flush_queue,
    hw_wait_for_state_transition,
    hw_current_state,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Telemetry rate used for all hardware tests.
TELEM_HZ: Final[int] = 10

# Number of telemetry frames to collect in basic parsing tests.
TELEM_SAMPLE_SIZE: Final[int] = 10

# Valid Arduino state names.
VALID_STATES: Final[frozenset[str]] = frozenset(
    {"IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"}
)

# Valid Arduino phase names.
VALID_PHASES: Final[frozenset[str]] = frozenset(
    {"NONE", "DESCENDING", "DWELL_BOTTOM", "ASCENDING", "DWELL_TOP"}
)

# ---------------------------------------------------------------------------
# Shared test profiles
# ---------------------------------------------------------------------------

# Shallow, slow, single-dip trapezoidal profile for a quick run test.
_TRAPEZOIDAL_PROFILE: Final[DipProfile] = DipProfile(
    name                = "hw_test_trapezoidal",
    dip_speed_mm_s      = 5.0,
    withdraw_speed_mm_s = 8.0,
    accel_mm_s2         = 30.0,
    dip_depth_mm        = 20.0,
    dwell_bottom_ms     = 1000,
    dwell_top_ms        = 500,
    n_dips              = 1,
    notes               = "Hardware test — trapezoidal profile",
)

# Multi-speed segmented profile: slow entry, fast bulk descent, soak,
# slow drainage withdraw, drainage pause, fast full withdraw.
_SEGMENTED_PROFILE: Final[DipProfile] = DipProfile(
    name                  = "hw_test_segmented",
    dip_speed_mm_s        = 5.0,
    withdraw_speed_mm_s   = 8.0,
    accel_mm_s2           = 30.0,
    dip_depth_mm          = 20.0,
    dwell_bottom_ms       = 2000,
    dwell_top_ms          = 500,
    n_dips                = 1,
    notes                 = "Hardware test — segmented profile",
    velocity_profile_type = "segmented",
    velocity_profile_data = {
        "segments": [
            {
                "type": "move", "distance_mm": -5.0,
                "speed_mm_s": 3.0, "accel_mm_s2": 20.0,
                "comment": "slow entry",
            },
            {
                "type": "move", "distance_mm": -15.0,
                "speed_mm_s": 10.0, "accel_mm_s2": 30.0,
                "comment": "fast bulk descent",
            },
            {
                "type": "dwell", "duration_ms": 2000,
                "comment": "soak at bottom",
            },
            {
                "type": "move", "distance_mm": 5.0,
                "speed_mm_s": 2.0, "accel_mm_s2": 20.0,
                "comment": "slow drainage withdraw",
            },
            {
                "type": "dwell", "duration_ms": 1000,
                "comment": "drainage pause",
            },
            {
                "type": "move", "distance_mm": 15.0,
                "speed_mm_s": 12.0, "accel_mm_s2": 30.0,
                "comment": "fast full withdraw",
            },
        ]
    },
)


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _collect_frames(
    mgr: SerialManager,
    n: int,
    timeout_s: float = 10.0,
) -> list[TelemetryFrame]:
    """Collect up to *n* telemetry frames, returning whatever arrived."""
    frames: list[TelemetryFrame] = []
    deadline: float = time.monotonic() + timeout_s
    while len(frames) < n and time.monotonic() < deadline:
        try:
            frame: TelemetryFrame = mgr.telem_queue.get(
                timeout=min(deadline - time.monotonic(), 1.0)
            )
            frames.append(frame)
        except queue.Empty:
            pass
    return frames


# ---------------------------------------------------------------------------
# Tests — connection and telemetry
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestConnection:
    """Verify that the serial connection and telemetry pipeline work."""

    def test_manager_is_connected(self, hw_manager: SerialManager) -> None:
        """SerialManager.is_connected returns True after start()."""
        assert hw_manager.is_connected

    def test_telemetry_frames_arrive(self, hw_manager: SerialManager) -> None:
        """At least one telemetry frame arrives within 5 seconds of connecting."""
        frames: list[TelemetryFrame] = _collect_frames(hw_manager, n=1, timeout_s=5.0)
        assert len(frames) >= 1, "No telemetry frames received — check baud rate and firmware"

    def test_set_telem_rate(self, hw_manager: SerialManager, hw_ci: CommandInterface) -> None:
        """SET_TELEM_RATE command is acknowledged without error."""
        hw_ci.set_telem_rate(TELEM_HZ)   # raises CommandError on failure

    def test_frames_parse_correctly(self, hw_manager: SerialManager) -> None:
        """All sampled frames have valid state/phase strings and finite numeric fields."""
        import math

        frames: list[TelemetryFrame] = _collect_frames(
            hw_manager, n=TELEM_SAMPLE_SIZE, timeout_s=10.0
        )
        assert len(frames) >= TELEM_SAMPLE_SIZE, (
            f"Only received {len(frames)}/{TELEM_SAMPLE_SIZE} frames"
        )

        frame: TelemetryFrame
        for frame in frames:
            assert frame.state in VALID_STATES, (
                f"Unexpected state {frame.state!r}"
            )
            assert frame.phase in VALID_PHASES, (
                f"Unexpected phase {frame.phase!r}"
            )
            assert math.isfinite(frame.pos_mm), (
                f"pos_mm is not finite: {frame.pos_mm}"
            )
            assert math.isfinite(frame.vel_actual_mm_s), (
                f"vel_actual_mm_s is not finite: {frame.vel_actual_mm_s}"
            )
            assert math.isfinite(frame.vel_commanded_mm_s), (
                f"vel_commanded_mm_s is not finite: {frame.vel_commanded_mm_s}"
            )
            assert frame.timestamp_ms >= 0, (
                f"timestamp_ms is negative: {frame.timestamp_ms}"
            )

    def test_response_queue_empty_at_rest(self, hw_manager: SerialManager) -> None:
        """Response queue has no stale ACK/ERR messages sitting in it at rest."""
        # Give any in-flight responses 0.5 s to arrive, then check.
        time.sleep(0.5)
        assert hw_manager.response_queue.empty(), (
            "Unexpected data in response queue — may indicate protocol mismatch"
        )


# ---------------------------------------------------------------------------
# Tests — commands
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestCommands:
    """Verify that individual commands are acknowledged correctly."""

    def test_set_telem_rate_zero_then_restore(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """SET_TELEM_RATE 0 stops telemetry; restoring to TELEM_HZ resumes it."""
        hw_ci.set_telem_rate(0)

        # Drain any frames that arrived before the rate was set to 0.
        hw_flush_queue(hw_manager)
        time.sleep(1.0)

        # No frames should arrive while rate is 0.
        frames_while_silent: list[TelemetryFrame] = _collect_frames(
            hw_manager, n=1, timeout_s=1.5
        )
        assert len(frames_while_silent) == 0, (
            "Frames arrived after SET_TELEM_RATE 0 — firmware may not honour rate=0"
        )

        # Restore and verify frames resume.
        hw_ci.set_telem_rate(TELEM_HZ)
        frames_after_restore: list[TelemetryFrame] = _collect_frames(
            hw_manager, n=3, timeout_s=5.0
        )
        assert len(frames_after_restore) >= 3, (
            "Telemetry did not resume after SET_TELEM_RATE restored"
        )

    def test_estop_acknowledged(self, hw_ci: CommandInterface) -> None:
        """CMD ESTOP is acknowledged — does not raise CommandError."""
        # ESTOP from READY is valid; Arduino should ACK immediately.
        # Note: this may put the Arduino in ERROR state — home is required after.
        try:
            hw_ci.estop()
        except CommandError as exc:
            pytest.fail(f"ESTOP raised CommandError: {exc}")


# ---------------------------------------------------------------------------
# Tests — homing
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestHoming:
    """Verify homing brings the Arduino to READY state."""

    def test_home_reaches_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """CMD HOME is acknowledged and the Arduino reaches READY state."""
        hw_ci.home()
        hw_flush_queue(hw_manager)

        reached_ready: bool = hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=60.0,
        )
        assert reached_ready, "Arduino did not reach READY after homing"

    def test_state_is_ready_after_home(
        self,
        hw_manager: SerialManager,
    ) -> None:
        """The next telemetry frame after homing reports state=READY."""
        state: str = hw_current_state(hw_manager, timeout_s=5.0)
        assert state == "READY", f"Expected READY after home, got {state!r}"


# ---------------------------------------------------------------------------
# Tests — trapezoidal profile run
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestTrapezoidalRun:
    """Run a trapezoidal profile and verify telemetry and DataRecorder output."""

    def test_trapezoidal_run_completes(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Trapezoidal RUN_PROFILE completes and Arduino returns to READY."""
        # Ensure we start from READY.
        hw_ci.home()
        hw_flush_queue(hw_manager)
        assert hw_wait_for_state_transition(
            hw_manager, "READY", "READY", timeout_leave_s=5.0, timeout_arrive_s=60.0
        ), "Could not reach READY before trapezoidal run"

        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(_TRAPEZOIDAL_PROFILE.name)

        hw_ci.run(_TRAPEZOIDAL_PROFILE)
        hw_flush_queue(hw_manager)

        completed: bool = hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        )
        assert completed, "Trapezoidal run did not complete within timeout"

        run: RecordedRun = recorder.finish()
        assert run.frame_count > 0, "DataRecorder recorded no frames"

    def test_trapezoidal_run_records_position(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Position moves beyond 1 mm during a trapezoidal run."""
        import numpy as np

        hw_ci.home()
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY", timeout_leave_s=5.0, timeout_arrive_s=60.0
        )

        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(_TRAPEZOIDAL_PROFILE.name)

        hw_ci.run(_TRAPEZOIDAL_PROFILE)
        hw_flush_queue(hw_manager)

        hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        )

        run: RecordedRun = recorder.finish()
        pos: Float64Array = run.arrays["pos_mm"]
        peak_displacement: float = float(np.abs(pos).max())

        assert peak_displacement > 1.0, (
            f"Peak displacement {peak_displacement:.2f} mm — motor may not have moved"
        )

    def test_trapezoidal_run_csv_written(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """DataRecorder writes a non-empty CSV file after the run."""
        hw_ci.home()
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY", timeout_leave_s=5.0, timeout_arrive_s=60.0
        )

        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(_TRAPEZOIDAL_PROFILE.name)

        hw_ci.run(_TRAPEZOIDAL_PROFILE)
        hw_flush_queue(hw_manager)

        hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        )

        run: RecordedRun = recorder.finish()
        assert run.csv_path is not None, "No CSV path returned"
        from pathlib import Path
        csv_file: Path = Path(run.csv_path)
        assert csv_file.exists(), f"CSV file not found: {csv_file}"
        lines: list[str] = csv_file.read_text().splitlines()
        assert len(lines) > 1, "CSV file contains only the header — no data rows"


# ---------------------------------------------------------------------------
# Tests — segmented profile run
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestSegmentedRun:
    """Stream a segmented profile and verify phase transitions and telemetry."""

    def test_segmented_run_completes(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Segmented RUN_LOADED_MOVE completes and Arduino returns to READY."""
        hw_ci.home()
        hw_flush_queue(hw_manager)
        assert hw_wait_for_state_transition(
            hw_manager, "READY", "READY", timeout_leave_s=5.0, timeout_arrive_s=60.0
        ), "Could not reach READY before segmented run"

        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(_SEGMENTED_PROFILE.name)

        hw_ci.run(_SEGMENTED_PROFILE)
        hw_flush_queue(hw_manager)

        completed: bool = hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        )
        assert completed, "Segmented run did not complete within timeout"

        run: RecordedRun = recorder.finish()
        assert run.frame_count > 0, "DataRecorder recorded no frames during segmented run"

    def test_segmented_run_velocity_varies(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Actual velocity covers at least two distinct speed bands during the run.

        The segmented profile uses 3.0 mm/s entry and 10.0 mm/s bulk descent.
        The actual velocity should span a range of at least 5 mm/s, confirming
        the Arduino executed the variable-speed segments rather than a single
        constant-speed move.
        """
        import numpy as np

        hw_ci.home()
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY", timeout_leave_s=5.0, timeout_arrive_s=60.0
        )

        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(_SEGMENTED_PROFILE.name)

        hw_ci.run(_SEGMENTED_PROFILE)
        hw_flush_queue(hw_manager)

        hw_wait_for_state_transition(
            hw_manager,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        )

        run: RecordedRun = recorder.finish()
        vel: Float64Array = run.arrays["vel_actual_mm_s"]
        vel_range: float = float(np.abs(vel).max() - np.abs(vel).min())

        assert vel_range > 5.0, (
            f"Velocity range {vel_range:.2f} mm/s — expected > 5.0 mm/s for "
            "variable-speed segmented profile"
        )


# ---------------------------------------------------------------------------
# Tests — TrapezoidalProfile against hardware
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestTrapezoidalProfileHardware:
    """Validate TrapezoidalProfile kinematics against real motor telemetry.

    These tests construct a :class:`~motion.trapezoidal_profile.TrapezoidalProfile`,
    run it on the Arduino, and compare the recorded telemetry to the
    analytically predicted values.  They also verify the segment-streaming
    path (``to_segments()`` → ``BEGIN_SEGMENTED_MOVE``) produces the same
    motion as the direct ``CMD RUN_PROFILE`` shortcut.
    """

    # Profile used across the class — slow enough to be safe, fast enough
    # to produce clear velocity data.
    _SPEED:  float = 8.0    # mm/s
    _ACCEL:  float = 30.0   # mm/s²
    _DIST:   float = 20.0   # mm

    def _home_and_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """Home the Arduino and block until READY.  Fails the test on timeout."""
        hw_ci.home()
        hw_flush_queue(hw_manager)
        reached: bool = hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=60.0,
        )
        assert reached, "Could not reach READY before test"

    def _run_profile_and_record(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        profile: DipProfile,
        log_dir: str,
    ) -> RecordedRun:
        """Run *profile*, record all telemetry, and return the completed RecordedRun."""
        recorder: DataRecorder = DataRecorder(log_dir=log_dir)
        recorder.start(profile.name)
        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=120.0,
            recorder=recorder,
        )
        return recorder.finish()

    # ------------------------------------------------------------------
    # Buffer / analytical checks (no hardware motion needed)
    # ------------------------------------------------------------------

    def test_profile_buffer_check(self) -> None:
        """Profile with 1 mm segments fits in the 64-segment Arduino buffer."""
        p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = self._SPEED,
            accel_mm_s2       = self._ACCEL,
            distance_mm       = -self._DIST,
        )
        assert p.fits_in_arduino_buffer(segment_length_mm=1.0), (
            f"Profile produces {p.segment_count(1.0)} segments — exceeds buffer of 64"
        )

    def test_segment_distances_sum_to_profile_distance(self) -> None:
        """to_segments() distances sum exactly to the total profile distance."""
        p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = self._SPEED,
            accel_mm_s2       = self._ACCEL,
            distance_mm       = -self._DIST,
        )
        segs: list[MoveSegment] = p.to_segments(1.0)
        total: float          = sum(abs(s.distance_mm) for s in segs)
        expected_dist: float  = self._DIST
        assert total == pytest.approx(expected_dist, rel=1e-6)

    # ------------------------------------------------------------------
    # Hardware motion tests
    # ------------------------------------------------------------------

    def test_run_profile_shortcut_peak_velocity(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Peak recorded velocity is within 20 % of target speed during RUN_PROFILE.

        Uses the CMD RUN_PROFILE shortcut (Arduino computes motion natively).
        A 20 % tolerance accounts for the discrete 10 Hz telemetry sampling
        potentially missing the exact cruise peak.
        """
        self._home_and_ready(hw_manager, hw_ci)

        profile: DipProfile = DipProfile(
            name                = "hw_trap_shortcut",
            dip_speed_mm_s      = self._SPEED,
            withdraw_speed_mm_s = self._SPEED,
            accel_mm_s2         = self._ACCEL,
            dip_depth_mm        = self._DIST,
            dwell_bottom_ms     = 500,
            dwell_top_ms        = 0,
            n_dips              = 1,
            velocity_profile_type = "trapezoidal",
        )
        run: RecordedRun = self._run_profile_and_record(
            hw_manager, hw_ci, profile, str(tmp_path)
        )

        vel: Float64Array   = run.arrays["vel_actual_mm_s"]
        peak_vel: float     = float(np.abs(vel).max())
        tolerance: float    = 0.20 * self._SPEED

        assert peak_vel > 0.0, "No velocity recorded — motor may not have moved"
        assert abs(peak_vel - self._SPEED) < tolerance, (
            f"Peak velocity {peak_vel:.2f} mm/s is not within 20 % of "
            f"target {self._SPEED} mm/s"
        )

    def test_run_via_segments_reaches_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Streaming to_segments() via BEGIN_SEGMENTED_MOVE completes successfully.

        Converts a TrapezoidalProfile to MoveSegments, wraps them in a
        segmented DipProfile, and runs via the segmented command path.
        Verifies the Arduino returns to READY.
        """
        self._home_and_ready(hw_manager, hw_ci)

        p: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = self._SPEED,
            accel_mm_s2       = self._ACCEL,
            distance_mm       = -self._DIST,
        )
        # 1 mm segments → ~20 commands for a 20 mm move.  The inter-segment
        # delay in _run_segmented() prevents RX buffer overflow.
        segs: list[MoveSegment] = p.to_segments(segment_length_mm=1.0)
        seg_dicts: list[dict[str, object]] = [
            {
                "type":        "move",
                "distance_mm": float(s.distance_mm),
                "speed_mm_s":  float(s.speed_mm_s),
                "accel_mm_s2": float(s.accel_mm_s2),
            }
            for s in segs
        ]

        profile: DipProfile = DipProfile(
            name                  = "hw_trap_streamed",
            dip_speed_mm_s        = self._SPEED,
            withdraw_speed_mm_s   = self._SPEED,
            accel_mm_s2           = self._ACCEL,
            dip_depth_mm          = self._DIST,
            dwell_bottom_ms       = 0,
            dwell_top_ms          = 0,
            n_dips                = 1,
            velocity_profile_type = "segmented",
            velocity_profile_data = {"segments": seg_dicts},
        )

        run: RecordedRun = self._run_profile_and_record(
            hw_manager, hw_ci, profile, str(tmp_path)
        )
        assert run.frame_count > 0, "No telemetry recorded during streamed run"

    def test_duration_within_tolerance(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Measured run duration is within 30 % of the analytically predicted value.

        30 % tolerance accounts for dwell time, Arduino scheduling jitter,
        and 10 Hz telemetry resolution.
        """
        self._home_and_ready(hw_manager, hw_ci)

        # RUN_PROFILE executes a full dip cycle: descent + dwell_bottom +
        # ascent + dwell_top.  Predict the duration for both legs separately.
        descent: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = self._SPEED,
            accel_mm_s2       = self._ACCEL,
            distance_mm       = -self._DIST,
        )
        ascent: TrapezoidalProfile = TrapezoidalProfile(
            target_speed_mm_s = self._SPEED,
            accel_mm_s2       = self._ACCEL,
            distance_mm       = self._DIST,
        )
        dwell_bottom_s: float = 0.0
        dwell_top_s:    float = 0.0
        predicted_s:    float = (
            descent.total_duration()
            + dwell_bottom_s
            + ascent.total_duration()
            + dwell_top_s
        )

        profile: DipProfile = DipProfile(
            name                = "hw_trap_duration",
            dip_speed_mm_s      = self._SPEED,
            withdraw_speed_mm_s = self._SPEED,
            accel_mm_s2         = self._ACCEL,
            dip_depth_mm        = self._DIST,
            dwell_bottom_ms     = 0,
            dwell_top_ms        = 0,
            n_dips              = 1,
            velocity_profile_type = "trapezoidal",
        )

        t_start: float   = time.monotonic()
        self._run_profile_and_record(hw_manager, hw_ci, profile, str(tmp_path))
        measured_s: float = time.monotonic() - t_start

        # 30 % tolerance covers serial latency, state-detection overhead,
        # and Arduino scheduling jitter on top of the pure motion time.
        tolerance: float = 0.30 * predicted_s
        assert abs(measured_s - predicted_s) < tolerance, (
            f"Measured duration {measured_s:.2f} s deviates from predicted "
            f"{predicted_s:.2f} s by more than 30 %"
        )


# ---------------------------------------------------------------------------
# Tests — ParabolicProfile against hardware
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestParabolicProfileHardware:
    """Validate ParabolicProfile kinematics against real motor telemetry.

    These tests stream a :class:`~motion.parabolic_profile.ParabolicProfile`
    via the ``BEGIN_SEGMENTED_MOVE`` path and verify that the recorded velocity
    follows the expected parabolic bell-curve shape — peaking near the midpoint
    of travel and starting/ending at a substantially lower speed.
    """

    _SPEED: float = 8.0    # mm/s  — peak speed at the profile midpoint
    _ACCEL: float = 30.0   # mm/s²
    _DIST:  float = 20.0   # mm

    # ------------------------------------------------------------------
    # Shared helpers (mirrors TestTrapezoidalProfileHardware)
    # ------------------------------------------------------------------

    def _home_and_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        hw_ci.home()
        hw_flush_queue(hw_manager)
        reached: bool = hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=60.0,
        )
        assert reached, "Could not reach READY before parabolic test"

    def _build_profile(self, seg_len: float = 1.0) -> DipProfile:
        """Wrap a ParabolicProfile in a DipProfile ready for hw_ci.run()."""
        p: ParabolicProfile = ParabolicProfile(
            target_speed_mm_s = self._SPEED,
            distance_mm       = -self._DIST,
            accel_mm_s2       = self._ACCEL,
        )
        segs: list[MoveSegment] = p.to_segments(seg_len)
        seg_dicts: list[dict[str, object]] = [
            {
                "type":        "move",
                "distance_mm": float(s.distance_mm),
                "speed_mm_s":  float(s.speed_mm_s),
                "accel_mm_s2": float(s.accel_mm_s2),
            }
            for s in segs
        ]
        return DipProfile(
            name                  = "hw_parabolic",
            dip_speed_mm_s        = self._SPEED,
            withdraw_speed_mm_s   = self._SPEED,
            accel_mm_s2           = self._ACCEL,
            dip_depth_mm          = self._DIST,
            dwell_bottom_ms       = 0,
            dwell_top_ms          = 0,
            n_dips                = 1,
            velocity_profile_type = "segmented",
            velocity_profile_data = {"segments": seg_dicts},
        )

    def _run_and_record(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        log_dir: str,
        seg_len: float = 1.0,
    ) -> RecordedRun:
        profile: DipProfile = self._build_profile(seg_len)
        recorder: DataRecorder = DataRecorder(log_dir=log_dir)
        recorder.start(profile.name)
        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=180.0,
            recorder=recorder,
        )
        return recorder.finish()

    # ------------------------------------------------------------------
    # Static checks (no hardware motion)
    # ------------------------------------------------------------------

    def test_profile_fits_in_arduino_buffer(self) -> None:
        """20 mm / 1 mm = 20 segments — well within the 64-slot buffer."""
        p: ParabolicProfile = ParabolicProfile(
            target_speed_mm_s = self._SPEED,
            distance_mm       = -self._DIST,
            accel_mm_s2       = self._ACCEL,
        )
        assert p.fits_in_arduino_buffer(segment_length_mm=1.0), (
            f"Profile produces {p.segment_count(1.0)} segments — exceeds buffer of 64"
        )

    def test_segment_distances_sum_to_profile_distance(self) -> None:
        """to_segments() distances sum exactly to the total profile distance."""
        p: ParabolicProfile = ParabolicProfile(
            target_speed_mm_s = self._SPEED,
            distance_mm       = -self._DIST,
            accel_mm_s2       = self._ACCEL,
        )
        segs: list[MoveSegment] = p.to_segments(1.0)
        total: float = sum(abs(s.distance_mm) for s in segs)
        assert total == pytest.approx(self._DIST, rel=1e-6)

    # ------------------------------------------------------------------
    # Hardware motion tests
    # ------------------------------------------------------------------

    def test_run_parabolic_reaches_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Streaming a ParabolicProfile via BEGIN_SEGMENTED_MOVE completes
        successfully and the Arduino returns to READY."""
        self._home_and_ready(hw_manager, hw_ci)
        run: RecordedRun = self._run_and_record(hw_manager, hw_ci, str(tmp_path))
        assert run.frame_count > 0, "No telemetry recorded during parabolic run"

    def test_parabolic_peak_velocity(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Peak recorded velocity is within 20 % of the target peak speed.

        A 20 % tolerance accounts for 10 Hz telemetry possibly missing the
        exact peak frame and for hardware acceleration ramp settling time.
        """
        self._home_and_ready(hw_manager, hw_ci)
        run: RecordedRun = self._run_and_record(hw_manager, hw_ci, str(tmp_path))

        vel: Float64Array = run.arrays["vel_actual_mm_s"]
        peak_vel: float   = float(np.abs(vel).max())
        tolerance: float  = 0.20 * self._SPEED

        assert peak_vel > 0.0, "No velocity recorded — motor may not have moved"
        assert abs(peak_vel - self._SPEED) < tolerance, (
            f"Peak velocity {peak_vel:.2f} mm/s is not within 20 % of "
            f"target {self._SPEED} mm/s"
        )

    def test_parabolic_velocity_starts_slow(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Early recorded velocity is significantly below the peak speed.

        The parabolic profile starts near 0 mm/s and accelerates to the peak
        at the midpoint.  The first quarter of recorded frames should have a
        maximum speed below 50 % of the peak — this would not hold for a
        trapezoidal profile, which reaches cruise speed within ~1 mm.
        """
        self._home_and_ready(hw_manager, hw_ci)
        run: RecordedRun = self._run_and_record(hw_manager, hw_ci, str(tmp_path))

        vel: Float64Array    = run.arrays["vel_actual_mm_s"]
        vel_abs: Float64Array = np.abs(vel)

        # Only consider frames where the motor is actually moving (|v| > 0.1).
        moving_mask = vel_abs > 0.1
        if moving_mask.sum() < 8:
            pytest.skip("Too few moving frames to evaluate velocity shape")

        moving_vel: Float64Array = vel_abs[moving_mask]

        # First quarter of moving frames should be well below the peak.
        n_early: int             = max(1, len(moving_vel) // 4)
        early_max: float         = float(moving_vel[:n_early].max())
        peak_vel: float          = float(moving_vel.max())

        assert early_max < 0.5 * peak_vel, (
            f"Early-phase max velocity {early_max:.2f} mm/s is not below 50 % of "
            f"peak {peak_vel:.2f} mm/s — profile may not have the expected "
            "parabolic shape (slow entry)"
        )

    def test_parabolic_velocity_range_exceeds_trapezoidal_range(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """The parabolic profile uses a wider velocity range than a trapezoidal
        profile at the same peak speed.

        A trapezoidal profile spends most of its time at cruise speed, so
        |v_max − v_min| ≈ cruise − ramp_entry ≈ small.
        A parabolic profile ramps from near-zero to peak, so
        |v_max − v_min| spans almost the full [0, v_peak] range.
        """
        self._home_and_ready(hw_manager, hw_ci)
        run: RecordedRun = self._run_and_record(hw_manager, hw_ci, str(tmp_path))

        vel: Float64Array     = run.arrays["vel_actual_mm_s"]
        vel_abs: Float64Array = np.abs(vel)
        moving_mask           = vel_abs > 0.1
        if moving_mask.sum() < 8:
            pytest.skip("Too few moving frames to evaluate velocity range")

        moving_vel: Float64Array = vel_abs[moving_mask]
        vel_range: float         = float(moving_vel.max() - moving_vel.min())

        # The parabolic profile spans nearly the full speed range.
        # Require at least 60 % of v_peak covered.
        assert vel_range > 0.6 * self._SPEED, (
            f"Velocity range {vel_range:.2f} mm/s is less than 60 % of peak "
            f"{self._SPEED} mm/s — parabolic bell-curve shape not confirmed"
        )


# ---------------------------------------------------------------------------
# Tests — SplineProfile against hardware
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestSplineProfileHardware:
    """Validate SplineProfile kinematics against real motor telemetry.

    Two static tests run immediately (no motion, no hardware required):

    * :meth:`test_constructor_raises_not_implemented` — confirms the stub
      raises the expected exception at construction time.
    * :meth:`test_dispatch_raises_not_implemented` — confirms that
      ``CommandInterface.run()`` raises ``NotImplementedError`` for a profile
      with ``velocity_profile_type="spline"``, using ``MockArduino`` so no
      physical port is needed.

    Three hardware motion tests are marked ``skip`` and will be un-skipped
    in Step 23 once ``SplineProfile.to_segments()`` is implemented:

    * :meth:`test_run_spline_waypoints_reaches_ready`
    * :meth:`test_spline_follows_waypoint_peak_speed`
    * :meth:`test_spline_velocity_continuous`
    """

    # Waypoints: (position_mm, speed_mm_s).
    # Position is relative to home; negative = down into coating solution.
    _WAYPOINTS: list[tuple[float, float]] = [
        ( 0.0,  0.0),   # start at rest at home
        (-4.0,  2.0),   # slow entry into the solution meniscus
        (-10.0, 8.0),   # accelerate through bulk solution
        (-18.0, 3.0),   # decelerate as we approach target depth
        (-20.0, 0.0),   # come to rest at target depth
    ]
    _PEAK_SPEED: float = 8.0    # mm/s — speed at the fastest waypoint
    _DIST:       float = 20.0   # mm   — total descent depth
    _ACCEL:      float = 30.0   # mm/s² — acceleration limit passed to the profile

    # ------------------------------------------------------------------
    # Shared helpers
    # ------------------------------------------------------------------

    def _home_and_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        hw_ci.home()
        hw_flush_queue(hw_manager)
        reached: bool = hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=60.0,
        )
        assert reached, "Could not reach READY before spline test"

    def _build_profile(self) -> DipProfile:
        """Build a DipProfile with ``velocity_profile_type="spline"`` and waypoints.

        ``velocity_profile_data["waypoints"]`` holds the list of
        ``(position_mm, speed_mm_s)`` pairs that ``SplineProfile`` will fit
        an interpolant through once it is implemented.
        """
        return DipProfile(
            name                  = "hw_spline",
            dip_speed_mm_s        = self._PEAK_SPEED,
            withdraw_speed_mm_s   = self._PEAK_SPEED,
            accel_mm_s2           = self._ACCEL,
            dip_depth_mm          = self._DIST,
            dwell_bottom_ms       = 0,
            dwell_top_ms          = 0,
            n_dips                = 1,
            velocity_profile_type = "spline",
            velocity_profile_data = {"waypoints": list(self._WAYPOINTS)},
        )

    # ------------------------------------------------------------------
    # Static checks (no hardware motion) — run now
    # ------------------------------------------------------------------

    def test_constructor_raises_not_implemented(self) -> None:
        """SplineProfile raises NotImplementedError immediately on construction.

        The stub exists so imports never fail, but instantiation must raise
        a clear error rather than returning a broken object.
        """
        with pytest.raises(NotImplementedError, match="not yet implemented"):
            SplineProfile()

    def test_dispatch_raises_not_implemented(self) -> None:
        """CommandInterface.run() raises NotImplementedError for spline profiles.

        Verifies the ``"spline"`` branch in ``CommandInterface.run()`` hits the
        correct dispatch arm.  Uses ``MockArduino`` so no physical port is needed
        — the exception is raised before any serial command is sent.
        """
        from tests.mock_arduino import MockArduino

        mock: MockArduino = MockArduino()
        mock.start()
        try:
            ci: CommandInterface = CommandInterface(mock)  # type: ignore[arg-type]
            profile: DipProfile  = self._build_profile()
            with pytest.raises(NotImplementedError):
                ci.run(profile)
        finally:
            mock.stop()

    # ------------------------------------------------------------------
    # Hardware motion tests — skipped until SplineProfile is implemented
    # ------------------------------------------------------------------

    @pytest.mark.skip(reason="SplineProfile not yet implemented — un-skip in Step 23")
    def test_run_spline_waypoints_reaches_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Streaming a SplineProfile to hardware completes and Arduino returns
        to READY.

        When ``SplineProfile.to_segments()`` is implemented it will sample the
        fitted spline at ``segment_length_mm`` intervals and produce a
        ``MoveSegment`` list.  ``CommandInterface.run()`` will stream those
        segments through ``BEGIN_SEGMENTED_MOVE`` just like the parabolic path.
        """
        self._home_and_ready(hw_manager, hw_ci)

        profile:  DipProfile   = self._build_profile()
        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(profile.name)

        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        reached: bool = hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=180.0,
            recorder=recorder,
        )
        run: RecordedRun = recorder.finish()

        assert reached,           "Arduino did not return to READY after spline run"
        assert run.frame_count > 0, "No telemetry recorded during spline run"

    @pytest.mark.skip(reason="SplineProfile not yet implemented — un-skip in Step 23")
    def test_spline_follows_waypoint_peak_speed(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Peak recorded velocity is within 20 % of the fastest waypoint speed.

        The fastest waypoint in ``_WAYPOINTS`` specifies ``_PEAK_SPEED`` mm/s.
        A 20 % tolerance accounts for 10 Hz telemetry possibly missing the
        exact peak frame and for hardware acceleration ramp settling time.
        """
        self._home_and_ready(hw_manager, hw_ci)

        profile:  DipProfile   = self._build_profile()
        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(profile.name)

        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=180.0,
            recorder=recorder,
        )
        run: RecordedRun = recorder.finish()

        vel: Float64Array = run.arrays["vel_actual_mm_s"]
        peak_vel: float   = float(np.abs(vel).max())
        tolerance: float  = 0.20 * self._PEAK_SPEED

        assert peak_vel > 0.0, "No velocity recorded — motor may not have moved"
        assert abs(peak_vel - self._PEAK_SPEED) < tolerance, (
            f"Peak velocity {peak_vel:.2f} mm/s is not within 20 % of "
            f"waypoint peak {self._PEAK_SPEED} mm/s"
        )

    @pytest.mark.skip(reason="SplineProfile not yet implemented — un-skip in Step 23")
    def test_spline_velocity_continuous(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """No abrupt velocity jump exceeds 2 mm/s between consecutive telemetry
        frames during motion.

        Spline interpolation guarantees C1 continuity — the velocity curve must
        have no step discontinuities.  At 10 Hz telemetry, a 2 mm/s inter-frame
        jump corresponds to a 20 mm/s² transient, which is within the configured
        acceleration limit and therefore rules out firmware-level step changes
        between adjacent segments.
        """
        self._home_and_ready(hw_manager, hw_ci)

        profile:  DipProfile   = self._build_profile()
        recorder: DataRecorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(profile.name)

        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=180.0,
            recorder=recorder,
        )
        run: RecordedRun = recorder.finish()

        vel: Float64Array     = run.arrays["vel_actual_mm_s"]
        vel_abs: Float64Array = np.abs(vel)
        moving_mask           = vel_abs > 0.1
        if moving_mask.sum() < 4:
            pytest.skip("Too few moving frames to evaluate velocity continuity")

        moving_vel: Float64Array = np.abs(vel[moving_mask])
        diffs: Float64Array      = np.abs(np.diff(moving_vel))
        max_jump: float          = float(diffs.max())

        assert max_jump < 2.0, (
            f"Velocity jump of {max_jump:.2f} mm/s between consecutive frames "
            "exceeds 2 mm/s — spline interpolation may not be smooth"
        )


# ---------------------------------------------------------------------------
# Tests — live telemetry quality and pipeline
# ---------------------------------------------------------------------------

@pytest.mark.hardware
class TestTelemetryHardware:
    """Verify the live telemetry pipeline on real hardware.

    These tests focus on the *quality* and *correctness* of the data that
    arrives from the Arduino: timing accuracy, timestamp monotonicity,
    field consistency, phase transitions during a real run, commanded vs
    actual velocity separation, and the raw_queue feed that drives the
    serial monitor tab.

    All tests home first so that the Arduino is in READY state with a known
    position (0 mm at the bottom endstop) before any measurements are taken.
    """

    # Profile used throughout this class — short, slow, single dip.
    _DIP_SPEED:  float = 5.0    # mm/s
    _WITHDRAW:   float = 5.0    # mm/s
    _ACCEL:      float = 20.0   # mm/s²
    _DEPTH:      float = 20.0   # mm
    _DWELL_BOT:  int   = 500    # ms
    _DWELL_TOP:  int   = 200    # ms

    def _home_and_ready(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        hw_ci.home()
        hw_flush_queue(hw_manager)
        reached = hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=60.0,
        )
        assert reached, "Could not reach READY before telemetry test"

    def _run_and_collect(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> RecordedRun:
        """Home, run the short dip profile, record, and return RecordedRun."""
        self._home_and_ready(hw_manager, hw_ci)
        profile = DipProfile(
            name                = "hw_telem_test",
            dip_speed_mm_s      = self._DIP_SPEED,
            withdraw_speed_mm_s = self._WITHDRAW,
            accel_mm_s2         = self._ACCEL,
            dip_depth_mm        = self._DEPTH,
            dwell_bottom_ms     = self._DWELL_BOT,
            dwell_top_ms        = self._DWELL_TOP,
            n_dips              = 1,
        )
        recorder = DataRecorder(log_dir=str(tmp_path))
        recorder.start(profile.name)
        hw_ci.run(profile)
        hw_flush_queue(hw_manager)
        hw_wait_for_state_transition(
            hw_manager, "READY", "READY",
            timeout_leave_s=5.0, timeout_arrive_s=120.0,
            recorder=recorder,
        )
        return recorder.finish()

    # ------------------------------------------------------------------
    # Timing and rate accuracy
    # ------------------------------------------------------------------

    def test_telem_rate_approximately_correct(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """Inter-frame wall-clock intervals are within 50 % of the configured rate.

        50 % is intentionally loose — Python scheduling jitter and OS timer
        resolution mean exact 100 ms spacing is not guaranteed.  The test
        confirms the Arduino is broadcasting near the requested rate, not that
        timing is sub-millisecond precise.
        """
        hw_ci.set_telem_rate(TELEM_HZ)
        hw_flush_queue(hw_manager)

        n_frames = 15
        timestamps: list[float] = []
        deadline = time.monotonic() + 10.0

        while len(timestamps) < n_frames and time.monotonic() < deadline:
            try:
                hw_manager.telem_queue.get(timeout=0.5)
                timestamps.append(time.monotonic())
            except queue.Empty:
                pass

        assert len(timestamps) >= n_frames, (
            f"Only received {len(timestamps)}/{n_frames} frames within 10 s"
        )

        intervals = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
        expected_interval = 1.0 / TELEM_HZ
        tolerance = 0.50 * expected_interval   # ±50 %

        bad = [iv for iv in intervals if abs(iv - expected_interval) > tolerance]
        assert not bad, (
            f"{len(bad)}/{len(intervals)} inter-frame intervals outside ±50 % of "
            f"{expected_interval * 1000:.0f} ms: {[f'{v*1000:.0f}ms' for v in bad]}"
        )

    def test_timestamps_monotonically_increasing(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """Arduino millis() timestamps never decrease between consecutive frames.

        A non-monotonic timestamp indicates a firmware clock overflow or a
        frame being received out of order — neither should happen in normal
        operation.
        """
        hw_ci.set_telem_rate(TELEM_HZ)
        hw_flush_queue(hw_manager)

        frames = _collect_frames(hw_manager, n=20, timeout_s=10.0)
        assert len(frames) >= 10, f"Too few frames to test monotonicity: {len(frames)}"

        for i in range(1, len(frames)):
            assert frames[i].timestamp_ms >= frames[i - 1].timestamp_ms, (
                f"Timestamp went backwards at index {i}: "
                f"{frames[i-1].timestamp_ms} → {frames[i].timestamp_ms}"
            )

    # ------------------------------------------------------------------
    # Field validity at rest (READY state)
    # ------------------------------------------------------------------

    def test_position_near_backoff_after_home(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """Position after homing is within 2 mm of -HOMING_BACKOFF_MM (-5 mm).

        Homing sequence:
          1. Carriage moves up until the top endstop triggers.
          2. Encoder is zeroed at the top endstop (setHome()).
          3. Carriage backs off downward by HOMING_BACKOFF_MM = 5 mm.
          4. Arduino enters READY state; reported position ≈ -5 mm.

        Downward motion is negative in this coordinate system — dipping moves
        from -5 mm toward larger negative values (e.g. -25 mm for a 20 mm dip).
        Any deviation larger than 2 mm indicates a homing or encoder fault.
        """
        _BACKOFF_MM: float = 5.0   # must match HOMING_BACKOFF_MM in config.h
        self._home_and_ready(hw_manager, hw_ci)
        frame = hw_manager.telem_queue.get(timeout=5.0)
        assert abs(frame.pos_mm - (-_BACKOFF_MM)) < 2.0, (
            f"Position after home is {frame.pos_mm:.2f} mm — "
            f"expected near -{_BACKOFF_MM} mm (top endstop origin, backed off downward)"
        )

    def test_velocity_zero_at_rest(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """Both velocity fields are near zero when the motor is stationary."""
        self._home_and_ready(hw_manager, hw_ci)

        # Collect a few frames and check they are all stationary.
        frames = _collect_frames(hw_manager, n=5, timeout_s=5.0)
        assert frames, "No frames received after homing"

        for f in frames:
            assert abs(f.vel_actual_mm_s) < 0.5, (
                f"Actual velocity {f.vel_actual_mm_s:.2f} mm/s at rest"
            )
            assert abs(f.vel_commanded_mm_s) < 0.5, (
                f"Commanded velocity {f.vel_commanded_mm_s:.2f} mm/s at rest"
            )

    # ------------------------------------------------------------------
    # Phase transitions during a full dip cycle
    # ------------------------------------------------------------------

    def test_descending_phase_observed(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """At least one telemetry frame reports phase=DESCENDING during the run."""
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        assert "DESCENDING" in run.phases, (
            "DESCENDING phase never seen in telemetry — "
            "Arduino may not be reporting sub-phases"
        )

    def test_ascending_phase_observed(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """At least one telemetry frame reports phase=ASCENDING during the run."""
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        assert "ASCENDING" in run.phases, (
            "ASCENDING phase never seen in telemetry — "
            "run may have been interrupted or aborted early"
        )

    def test_dwell_bottom_phase_observed(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """DWELL_BOTTOM phase appears between DESCENDING and ASCENDING.

        Confirms the Arduino holds position for the configured dwell period
        rather than reversing immediately on reaching target depth.
        """
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        assert "DWELL_BOTTOM" in run.phases, (
            "DWELL_BOTTOM phase never seen — dwell may have been skipped"
        )

    def test_phase_order_is_correct(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Phase sequence follows DESCENDING → DWELL_BOTTOM → ASCENDING order.

        Extracts the *first occurrence* index of each key phase and asserts
        the expected ordering.  DWELL_TOP is optional (dwell_top_ms may be 0).
        """
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        phases = run.phases

        required = ("DESCENDING", "DWELL_BOTTOM", "ASCENDING")
        for phase in required:
            assert phase in phases, f"Phase {phase!r} not seen during run"

        idx_desc  = phases.index("DESCENDING")
        idx_dwell = phases.index("DWELL_BOTTOM")
        idx_asc   = phases.index("ASCENDING")

        assert idx_desc < idx_dwell < idx_asc, (
            f"Phase order incorrect: DESCENDING@{idx_desc}, "
            f"DWELL_BOTTOM@{idx_dwell}, ASCENDING@{idx_asc}"
        )

    # ------------------------------------------------------------------
    # Position and velocity during motion
    # ------------------------------------------------------------------

    def test_peak_displacement_matches_dip_depth(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Maximum recorded displacement from start position matches commanded dip depth.

        After homing the stage is at -HOMING_BACKOFF_MM (≈ -5 mm) — the encoder
        zeros at the top endstop and backs off downward.  A dip of depth D
        descends to approximately -(BACKOFF + D).  Travel relative to the start
        position is abs(min_pos - start_pos), which should equal D.

        15 % tolerance accounts for 10 Hz telemetry possibly missing the exact
        turnaround frame and for closed-loop encoder feedback settling.
        """
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        pos = run.arrays["pos_mm"]
        start_pos         = float(pos[0])
        peak_displacement = float(abs(pos.min() - start_pos))
        tolerance = 0.15 * self._DEPTH

        assert abs(peak_displacement - self._DEPTH) < tolerance, (
            f"Peak displacement {peak_displacement:.2f} mm deviates from "
            f"commanded depth {self._DEPTH} mm by more than 15 % "
            f"(start_pos={start_pos:.2f} mm, min_pos={pos.min():.2f} mm)"
        )

    def test_peak_velocity_within_tolerance(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Peak recorded velocity during descent is within 25 % of dip_speed_mm_s.

        25 % tolerance accounts for short 20 mm move where the motor may not
        fully reach cruise speed before beginning deceleration, and for 10 Hz
        sampling possibly missing the exact cruise peak.
        """
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        vel = run.arrays["vel_actual_mm_s"]
        peak_vel = float(np.abs(vel).max())
        tolerance = 0.25 * self._DIP_SPEED

        assert peak_vel > 0.0, "No velocity recorded — motor may not have moved"
        assert abs(peak_vel - self._DIP_SPEED) < tolerance, (
            f"Peak velocity {peak_vel:.2f} mm/s deviates from "
            f"dip_speed {self._DIP_SPEED} mm/s by more than 25 %"
        )

    def test_commanded_leads_actual_during_ramp(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
        tmp_path: pytest.TempPathFactory,
    ) -> None:
        """Commanded velocity exceeds actual velocity on at least one ramp frame.

        The uStepperS32 closed-loop PID means the motor takes a finite time to
        reach commanded speed.  During acceleration, vel_commanded_mm_s should
        exceed vel_actual_mm_s on at least a few frames.  If they are always
        equal, the firmware may be broadcasting the same value for both fields.
        """
        run = self._run_and_collect(hw_manager, hw_ci, tmp_path)
        vel_cmd = run.arrays["vel_commanded_mm_s"]
        vel_act = run.arrays["vel_actual_mm_s"]

        # Find frames where both are non-trivial (motor is actually moving).
        moving = np.abs(vel_act) > 0.5
        if moving.sum() < 3:
            pytest.skip("Too few moving frames to evaluate commanded vs actual")

        commanded_exceeds_actual = np.any(
            np.abs(vel_cmd[moving]) > np.abs(vel_act[moving]) + 0.3
        )
        assert commanded_exceeds_actual, (
            "Commanded velocity never exceeded actual velocity during motion. "
            "Both fields may be reporting the same value — check firmware telemetry."
        )

    # ------------------------------------------------------------------
    # raw_queue — serial monitor feed
    # ------------------------------------------------------------------

    def test_raw_queue_contains_tx_entries(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """raw_queue receives at least one TX entry after a command is sent.

        Verifies that SerialManager.send_command() puts a 'TX ...' string on
        raw_queue so the serial monitor tab has data to display.
        """
        # Drain any stale entries.
        while True:
            try:
                hw_manager.raw_queue.get_nowait()
            except queue.Empty:
                break

        hw_ci.set_telem_rate(TELEM_HZ)   # sends one command

        # Give the TX entry time to land on the queue.
        time.sleep(0.1)

        entries: list[str] = []
        while True:
            try:
                entries.append(hw_manager.raw_queue.get_nowait())
            except queue.Empty:
                break

        tx_entries = [e for e in entries if e.startswith("TX ")]
        assert tx_entries, (
            "No TX entries found in raw_queue after sending a command. "
            "SerialManager.send_command() may not be populating raw_queue."
        )

    def test_raw_queue_contains_rx_entries(
        self,
        hw_manager: SerialManager,
        hw_ci: CommandInterface,
    ) -> None:
        """raw_queue receives RX entries (ACK and TELEM) from the Arduino.

        Verifies the serial read loop is copying incoming lines to raw_queue
        so the serial monitor tab sees real Arduino output.
        """
        hw_ci.set_telem_rate(TELEM_HZ)
        hw_flush_queue(hw_manager)

        # Drain raw_queue of any stale entries.
        while True:
            try:
                hw_manager.raw_queue.get_nowait()
            except queue.Empty:
                break

        # Wait for a few telemetry frames so RX entries accumulate.
        _collect_frames(hw_manager, n=5, timeout_s=5.0)

        entries: list[str] = []
        while True:
            try:
                entries.append(hw_manager.raw_queue.get_nowait())
            except queue.Empty:
                break

        rx_telem = [e for e in entries if e.startswith("RX TELEM")]
        assert rx_telem, (
            "No RX TELEM entries in raw_queue. "
            "SerialManager._read_loop() may not be populating raw_queue."
        )
