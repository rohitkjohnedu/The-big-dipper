"""
tests/core/test_data_recorder.py
=================================

Tests for ``core/data_recorder.py`` — ``DataRecorder``, ``RecordedRun``,
and the ``_sanitise_filename`` helper.

Covers:
- Lifecycle: start → record → finish round-trip.
- Array correctness: values stored match values recorded.
- Array growth: recorder handles more than _CHUNK_SIZE frames.
- finish(save=False): no CSV written.
- finish() with 0 frames: no CSV written even when save=True.
- CSV file: created in log_dir, correct header, correct row values.
- current_arrays(): reflects live data mid-run.
- is_recording / frame_count properties.
- record() before start(): frame silently dropped.
- start() resets previous run data.
- _sanitise_filename: unsafe characters replaced, empty string handled.

All tests run without hardware — no serial port required.
"""

import csv
from pathlib import Path
from typing import Any, Final

import numpy as np
import pytest

from core.data_recorder import DataRecorder, Float64Array, RecordedRun, _sanitise_filename
from core.telemetry_parser import TelemetryFrame


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def make_frame(
    timestamp_ms:       int   = 1000,
    pos_mm:             float = 10.0,
    vel_actual_mm_s:    float = 5.0,
    vel_commanded_mm_s: float = 5.0,
    accel_mm_s2:        float = 0.0,
    state:              str   = "RUNNING",
    phase:              str   = "DESCENDING",
) -> TelemetryFrame:
    """
    Build a :class:`~core.telemetry_parser.TelemetryFrame` with sensible
    defaults, overridable per-field.

    Args:
        timestamp_ms:       Arduino millis() timestamp.
        pos_mm:             Encoder position in mm.
        vel_actual_mm_s:    Measured velocity in mm/s.
        vel_commanded_mm_s: Commanded velocity in mm/s.
        accel_mm_s2:        Acceleration in mm/s².
        state:              System state string.
        phase:              Run phase string.

    Returns:
        A :class:`~core.telemetry_parser.TelemetryFrame` instance.
    """
    return TelemetryFrame(
        timestamp_ms       = timestamp_ms,
        pos_mm             = pos_mm,
        vel_actual_mm_s    = vel_actual_mm_s,
        vel_commanded_mm_s = vel_commanded_mm_s,
        accel_mm_s2        = accel_mm_s2,
        state              = state,
        phase              = phase,
    )


# ---------------------------------------------------------------------------
# Lifecycle and properties
# ---------------------------------------------------------------------------

class TestLifecycle:
    """Verify the start → record → finish lifecycle."""

    def test_is_recording_false_before_start(self, tmp_path: Path) -> None:
        """is_recording must be False before start() is called."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        assert recorder.is_recording is False

    def test_is_recording_true_after_start(self, tmp_path: Path) -> None:
        """is_recording must be True after start() and before finish()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        assert recorder.is_recording is True

    def test_is_recording_false_after_finish(self, tmp_path: Path) -> None:
        """is_recording must be False after finish() is called."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.finish(save=False)
        assert recorder.is_recording is False

    def test_frame_count_zero_before_start(self, tmp_path: Path) -> None:
        """frame_count must be 0 before start()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        assert recorder.frame_count == 0

    def test_frame_count_increments_on_record(self, tmp_path: Path) -> None:
        """frame_count must increase by 1 for each recorded frame."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame())
        recorder.record(make_frame())
        assert recorder.frame_count == 2

    def test_frame_count_zero_after_finish(self, tmp_path: Path) -> None:
        """frame_count must reset to 0 after finish()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame())
        recorder.finish(save=False)
        assert recorder.frame_count == 0

    def test_start_resets_previous_data(self, tmp_path: Path) -> None:
        """A second start() must discard data from the previous run."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("run1")
        recorder.record(make_frame())
        recorder.record(make_frame())

        recorder.start("run2")
        assert recorder.frame_count == 0

    def test_record_before_start_is_dropped(self, tmp_path: Path) -> None:
        """record() before start() must not raise — the frame is silently dropped."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.record(make_frame())   # must not raise
        assert recorder.frame_count == 0


# ---------------------------------------------------------------------------
# Array correctness
# ---------------------------------------------------------------------------

class TestArrayValues:
    """Verify that recorded values end up in the correct array positions."""

    def test_position_stored_correctly(self, tmp_path: Path) -> None:
        """pos_mm values must be stored in the correct order."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(pos_mm=10.0))
        recorder.record(make_frame(pos_mm=20.5))
        recorder.record(make_frame(pos_mm=-5.0))
        run: RecordedRun = recorder.finish(save=False)

        expected: Float64Array = np.array([10.0, 20.5, -5.0])
        np.testing.assert_array_almost_equal(run.arrays["pos_mm"], expected)

    def test_timestamp_stored_correctly(self, tmp_path: Path) -> None:
        """timestamp_ms values must be stored as floats in order."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(timestamp_ms=0))
        recorder.record(make_frame(timestamp_ms=100))
        recorder.record(make_frame(timestamp_ms=200))
        run: RecordedRun = recorder.finish(save=False)

        expected: Float64Array = np.array([0.0, 100.0, 200.0])
        np.testing.assert_array_equal(run.arrays["timestamp_ms"], expected)

    def test_velocity_stored_correctly(self, tmp_path: Path) -> None:
        """vel_actual_mm_s must be stored correctly."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(vel_actual_mm_s=-10.0))
        run: RecordedRun = recorder.finish(save=False)

        expected_vel: float = -10.0
        assert run.arrays["vel_actual_mm_s"][0] == pytest.approx(expected_vel)

    def test_state_and_phase_stored_correctly(self, tmp_path: Path) -> None:
        """String fields must be stored in the states and phases lists."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(state="RUNNING", phase="DESCENDING"))
        recorder.record(make_frame(state="RUNNING", phase="DWELL_BOTTOM"))
        run: RecordedRun = recorder.finish(save=False)

        assert run.states == ["RUNNING", "RUNNING"]
        assert run.phases == ["DESCENDING", "DWELL_BOTTOM"]

    def test_frame_count_in_run(self, tmp_path: Path) -> None:
        """RecordedRun.frame_count must equal the number of recorded frames."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        n_frames: int = 7
        for _ in range(n_frames):
            recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=False)
        assert run.frame_count == n_frames

    def test_arrays_trimmed_to_frame_count(self, tmp_path: Path) -> None:
        """Returned arrays must have length == frame_count, not the allocated capacity."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        for _ in range(5):
            recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=False)

        # Each numeric array must be exactly 5 elements long.
        col: str
        for col in ("timestamp_ms", "pos_mm", "vel_actual_mm_s",
                    "vel_commanded_mm_s", "accel_mm_s2"):
            assert len(run.arrays[col]) == 5, f"{col} has wrong length"

    def test_profile_name_in_run(self, tmp_path: Path) -> None:
        """RecordedRun.profile_name must match the name passed to start()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("my_profile")
        run: RecordedRun = recorder.finish(save=False)
        assert run.profile_name == "my_profile"


# ---------------------------------------------------------------------------
# Buffer growth
# ---------------------------------------------------------------------------

class TestBufferGrowth:
    """Verify the recorder handles runs longer than the initial chunk size."""

    # Import the internal chunk size so the test stays in sync if it changes.
    from core.data_recorder import _CHUNK_SIZE as _CHUNK

    def test_records_more_than_chunk_size(self, tmp_path: Path) -> None:
        """Recording more frames than _CHUNK_SIZE must not raise or lose data."""
        from core.data_recorder import _CHUNK_SIZE

        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("long_run")

        n_frames: int = _CHUNK_SIZE + 50   # just over the initial allocation
        i: int
        for i in range(n_frames):
            recorder.record(make_frame(timestamp_ms=i, pos_mm=float(i)))

        run: RecordedRun = recorder.finish(save=False)

        assert run.frame_count == n_frames
        assert len(run.arrays["pos_mm"]) == n_frames
        # Verify a few specific values to confirm no data was lost or overwritten.
        assert run.arrays["pos_mm"][0] == pytest.approx(0.0)
        assert run.arrays["pos_mm"][_CHUNK_SIZE] == pytest.approx(float(_CHUNK_SIZE))
        assert run.arrays["pos_mm"][-1] == pytest.approx(float(n_frames - 1))


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

class TestCSVOutput:
    """Verify the CSV file written by finish(save=True)."""

    def test_csv_created_in_log_dir(self, tmp_path: Path) -> None:
        """A CSV file must be created inside log_dir on finish(save=True)."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("myprofile")
        recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is not None
        assert run.csv_path.exists()
        assert run.csv_path.parent == tmp_path

    def test_csv_filename_contains_profile_name(self, tmp_path: Path) -> None:
        """The CSV filename must contain the (sanitised) profile name."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("myprofile")
        recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is not None
        assert "myprofile" in run.csv_path.name

    def test_csv_header_correct(self, tmp_path: Path) -> None:
        """The CSV header must list all seven column names in the correct order."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is not None
        with run.csv_path.open(encoding="utf-8", newline="") as fh:
            reader: Any = csv.reader(fh)
            header: list[str] = next(reader)

        expected_header: list[str] = [
            "timestamp_ms", "pos_mm", "vel_actual_mm_s",
            "vel_commanded_mm_s", "accel_mm_s2", "state", "phase",
        ]
        assert header == expected_header

    def test_csv_row_values_correct(self, tmp_path: Path) -> None:
        """CSV rows must contain the exact values that were recorded."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(
            timestamp_ms       = 500,
            pos_mm             = -25.5,
            vel_actual_mm_s    = -10.0,
            vel_commanded_mm_s = -10.0,
            accel_mm_s2        = 0.0,
            state              = "RUNNING",
            phase              = "DESCENDING",
        ))
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is not None
        with run.csv_path.open(encoding="utf-8", newline="") as fh:
            reader: Any = csv.DictReader(fh)
            rows: list[dict[str, str]] = list(reader)

        assert len(rows) == 1
        row: dict[str, str] = rows[0]
        assert int(float(row["timestamp_ms"])) == 500
        assert float(row["pos_mm"])             == pytest.approx(-25.5)
        assert float(row["vel_actual_mm_s"])    == pytest.approx(-10.0)
        assert row["state"]                     == "RUNNING"
        assert row["phase"]                     == "DESCENDING"

    def test_csv_row_count_matches_frame_count(self, tmp_path: Path) -> None:
        """The number of data rows in the CSV must equal the number of frames recorded."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        n_frames: int = 15
        for _ in range(n_frames):
            recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is not None
        with run.csv_path.open(encoding="utf-8", newline="") as fh:
            reader: Any = csv.reader(fh)
            next(reader)   # skip header
            rows: list[list[str]] = list(reader)
        assert len(rows) == n_frames

    def test_no_csv_when_save_false(self, tmp_path: Path) -> None:
        """finish(save=False) must not write a CSV file."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame())
        run: RecordedRun = recorder.finish(save=False)

        assert run.csv_path is None
        csv_files: list[Path] = list(tmp_path.glob("*.csv"))
        assert csv_files == []

    def test_no_csv_when_zero_frames(self, tmp_path: Path) -> None:
        """finish(save=True) with 0 frames must not write a CSV file."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        run: RecordedRun = recorder.finish(save=True)

        assert run.csv_path is None
        csv_files: list[Path] = list(tmp_path.glob("*.csv"))
        assert csv_files == []

    def test_log_dir_created_if_absent(self, tmp_path: Path) -> None:
        """finish() must create log_dir if it does not already exist."""
        log_dir: Path = tmp_path / "nested" / "logs"
        assert not log_dir.exists()

        recorder: DataRecorder = DataRecorder(log_dir=log_dir)
        recorder.start("test")
        recorder.record(make_frame())
        recorder.finish(save=True)

        assert log_dir.exists()


# ---------------------------------------------------------------------------
# current_arrays (live view)
# ---------------------------------------------------------------------------

class TestCurrentArrays:
    """Verify the live array view returned by current_arrays()."""

    def test_empty_before_start(self, tmp_path: Path) -> None:
        """current_arrays() must return an empty dict before start()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        assert recorder.current_arrays() == {}

    def test_reflects_recorded_frames(self, tmp_path: Path) -> None:
        """current_arrays() must return only the frames recorded so far."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame(pos_mm=1.0))
        recorder.record(make_frame(pos_mm=2.0))

        live: dict[str, Float64Array] = recorder.current_arrays()
        expected_length: int = 2
        assert len(live["pos_mm"]) == expected_length
        assert live["pos_mm"][0] == pytest.approx(1.0)
        assert live["pos_mm"][1] == pytest.approx(2.0)

    def test_empty_after_finish(self, tmp_path: Path) -> None:
        """current_arrays() must return an empty dict after finish()."""
        recorder: DataRecorder = DataRecorder(log_dir=tmp_path)
        recorder.start("test")
        recorder.record(make_frame())
        recorder.finish(save=False)

        assert recorder.current_arrays() == {}


# ---------------------------------------------------------------------------
# _sanitise_filename
# ---------------------------------------------------------------------------

class TestSanitiseFilename:
    """Verify _sanitise_filename replaces unsafe characters correctly."""

    def test_spaces_replaced(self) -> None:
        """Spaces must be replaced with underscores."""
        result: str = _sanitise_filename("my profile")
        assert " " not in result

    def test_slashes_replaced(self) -> None:
        """Forward and back slashes must be replaced."""
        assert "/" not in _sanitise_filename("a/b")
        assert "\\" not in _sanitise_filename("a\\b")

    def test_safe_name_unchanged(self) -> None:
        """A name with no unsafe characters must be returned unchanged."""
        result: str = _sanitise_filename("default")
        assert result == "default"

    def test_empty_string_returns_unnamed(self) -> None:
        """An empty string must return 'unnamed'."""
        result: str = _sanitise_filename("")
        assert result == "unnamed"

    def test_whitespace_only_returns_unnamed(self) -> None:
        """A whitespace-only string must return 'unnamed'."""
        result: str = _sanitise_filename("   ")
        assert result == "unnamed"
