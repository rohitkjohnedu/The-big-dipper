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

from core.command_interface import CommandInterface, CommandError
from core.data_recorder import DataRecorder, Float64Array, RecordedRun
from core.profile import DipProfile
from core.serial_manager import SerialManager
from core.telemetry_parser import TelemetryFrame
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
