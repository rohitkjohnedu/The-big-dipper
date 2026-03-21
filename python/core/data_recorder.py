"""
core/data_recorder.py
=====================

Records telemetry frames from the Arduino into in-memory numpy arrays and
auto-saves a CSV file when a run completes.

Design overview
---------------
The data recorder sits between the serial thread and the live-plot widget.
During a run, each :class:`~core.telemetry_parser.TelemetryFrame` received
from ``telem_queue`` is passed to :meth:`record`.  All numeric fields are
appended to pre-allocated numpy arrays that grow dynamically in chunks.

When :meth:`finish` is called (run complete or stopped), the arrays are
trimmed to their actual length and written to a CSV file under ``logs/``.

The live-plot widget reads :attr:`arrays` at any time to render the current
data — no locking is needed because the UI thread is the only consumer and
the only writer (frames arrive via Qt signals on the main thread).

CSV file naming::

    logs/run_<profile_name>_<YYYYMMDD_HHMMSS>.csv

Columns (in order, matching :class:`~core.telemetry_parser.TelemetryFrame`)::

    timestamp_ms, pos_mm, vel_actual_mm_s, vel_commanded_mm_s,
    accel_mm_s2, state, phase

Usage example::

    recorder = DataRecorder(log_dir=Path("logs"))
    recorder.start("my_profile")

    # called once per telemetry frame from the UI thread:
    recorder.record(frame)

    # when the run ends:
    recorder.finish()     # trims arrays, writes CSV

    # access recorded data for plotting:
    t = recorder.arrays["timestamp_ms"]
    y = recorder.arrays["pos_mm"]
"""

import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Final, TypeAlias

import numpy as np
import numpy.typing as npt

from core.telemetry_parser import TelemetryFrame

# Type alias for a 1-D numpy array of 64-bit floats.  Used for all numeric
# telemetry columns stored by DataRecorder and returned in RecordedRun.
Float64Array: TypeAlias = npt.NDArray[np.float64]

# Module-level logger — messages appear under "core.data_recorder".
log: Final[logging.Logger] = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Internal constants
# ---------------------------------------------------------------------------

# Initial capacity for each numeric array (number of frames).  Arrays grow
# in this step when capacity is exceeded.  At 10 Hz over 10 minutes this is
# 6 000 frames — a 1 000-frame chunk adds negligible allocation overhead.
_CHUNK_SIZE: Final[int] = 1_000

# Names of all numeric columns stored in numpy arrays.  String fields (state,
# phase) are stored separately as plain Python lists.
_NUMERIC_COLS: Final[tuple[str, ...]] = (
    "timestamp_ms",
    "pos_mm",
    "vel_actual_mm_s",
    "vel_commanded_mm_s",
    "accel_mm_s2",
)

# String field names stored as Python lists (not numpy arrays).
_STRING_COLS: Final[tuple[str, ...]] = (
    "state",
    "phase",
)

# All column names in CSV output order.
_ALL_COLS: Final[tuple[str, ...]] = _NUMERIC_COLS + _STRING_COLS

# Sanitised characters not permitted in filenames on Windows or Linux.
_FILENAME_UNSAFE_CHARS: Final[str] = r'\/:*?"<>| '


# ---------------------------------------------------------------------------
# RecordedArrays dataclass-like container
# ---------------------------------------------------------------------------

class RecordedRun:
    """
    Immutable snapshot of one completed run's data.

    Returned by :meth:`DataRecorder.finish` and safe to read from any thread
    after the run ends — the arrays are trimmed copies, not views into the
    recorder's internal buffers.

    Attributes:
        profile_name: Name of the profile that produced this run.
        started_at:   UTC ISO-8601 timestamp of when :meth:`~DataRecorder.start`
                      was called.
        frame_count:  Number of telemetry frames recorded.
        arrays:       Dict mapping each numeric column name to a 1-D
                      ``numpy.ndarray`` of ``float64``.
        states:       List of ``state`` strings, one per frame.
        phases:       List of ``phase`` strings, one per frame.
        csv_path:     Path of the CSV file written by :meth:`~DataRecorder.finish`,
                      or ``None`` if the run was discarded without saving.
    """

    def __init__(
        self,
        profile_name: str,
        started_at:   str,
        frame_count:  int,
        arrays:       dict[str, Float64Array],
        states:       list[str],
        phases:       list[str],
        csv_path:     Path | None,
    ) -> None:
        """
        Construct a ``RecordedRun``.  All arguments are stored as-is.

        Args:
            profile_name: Name of the DipProfile that produced this run.
            started_at:   UTC ISO-8601 string from the moment recording began.
            frame_count:  Total frames stored in the arrays.
            arrays:       Dict of trimmed float64 numpy arrays, keyed by
                          column name (see :data:`_NUMERIC_COLS`).
            states:       List of state strings, length == frame_count.
            phases:       List of phase strings, length == frame_count.
            csv_path:     Filesystem path of the saved CSV, or ``None``.
        """
        self.profile_name: str                   = profile_name
        self.started_at:   str                   = started_at
        self.frame_count:  int                   = frame_count
        self.arrays:       dict[str, Float64Array] = arrays
        self.states:       list[str]             = states
        self.phases:       list[str]             = phases
        self.csv_path:     Path | None           = csv_path


# ---------------------------------------------------------------------------
# DataRecorder
# ---------------------------------------------------------------------------

class DataRecorder:
    """
    Accumulates telemetry frames during a run and saves them to CSV on
    completion.

    Lifecycle::

        recorder.start("profile_name")    # reset buffers, note start time
        for frame in ...:
            recorder.record(frame)        # append one frame
        run = recorder.finish()           # trim, save CSV, return RecordedRun

    Calling :meth:`start` again resets everything, so the recorder can be
    reused across multiple runs without re-instantiation.

    The recorder is **not** thread-safe.  All calls must come from the same
    thread (the UI/main thread, which receives telemetry via Qt signals).

    Attributes:
        log_dir:       Directory where CSV files are written.  Created
                       automatically on first :meth:`finish` call if absent.
        _profile_name: Name of the profile currently being recorded.
        _started_at:   UTC ISO-8601 timestamp of the current run's start.
        _count:        Number of frames recorded in the current run.
        _capacity:     Current allocated length of the internal arrays.
        _bufs:         Dict of pre-allocated float64 numpy arrays, keyed by
                       column name.  Length == _capacity; valid data occupies
                       indices 0.._count-1.
        _states:       List of state strings for the current run.
        _phases:       List of phase strings for the current run.
    """

    def __init__(self, log_dir: str | Path = "logs") -> None:
        """
        Initialise the recorder.

        Args:
            log_dir: Directory for CSV output files.  The directory is
                     created on the first :meth:`finish` call if it does
                     not already exist.  Defaults to ``"logs"`` relative
                     to the working directory.
        """
        self.log_dir: Path = Path(log_dir)

        # --- Current-run state (populated by start()) ------------------------
        self._profile_name: str       = ""
        self._started_at:   str       = ""
        self._count:        int       = 0
        self._capacity:     int       = 0

        # Pre-allocated numpy buffers and string lists — empty until start().
        self._bufs:   dict[str, Float64Array] = {}
        self._states: list[str]             = []
        self._phases: list[str]             = []

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------

    @property
    def frame_count(self) -> int:
        """
        Number of frames recorded in the current run.

        Returns ``0`` when no run is active (before the first :meth:`start`
        call or after :meth:`finish`).
        """
        return self._count

    @property
    def is_recording(self) -> bool:
        """
        ``True`` if :meth:`start` has been called and :meth:`finish` has not
        yet been called for the current run.
        """
        return bool(self._profile_name)

    def start(self, profile_name: str) -> None:
        """
        Reset the recorder and begin a new run.

        Discards any previously recorded data without saving.  Call
        :meth:`finish` *before* :meth:`start` if you want to save the
        previous run's data.

        Args:
            profile_name: Name of the profile being executed.  Used as part
                          of the CSV filename.  Non-filesystem-safe characters
                          are replaced with underscores.
        """
        # Sanitise the profile name for use in filenames.
        self._profile_name = _sanitise_filename(profile_name)
        self._started_at   = datetime.now(timezone.utc).isoformat()
        self._count        = 0
        self._capacity     = _CHUNK_SIZE

        # Allocate initial numpy buffers — all zeros, will be overwritten.
        self._bufs = {
            col: np.zeros(_CHUNK_SIZE, dtype=np.float64)
            for col in _NUMERIC_COLS
        }
        self._states = []
        self._phases = []

        log.debug(
            "DataRecorder started for profile %r at %s",
            self._profile_name,
            self._started_at,
        )

    def record(self, frame: TelemetryFrame) -> None:
        """
        Append one telemetry frame to the current run's buffers.

        If the internal arrays are full they are grown by :data:`_CHUNK_SIZE`
        before appending.  This keeps the amortised cost of each append O(1).

        Args:
            frame: A :class:`~core.telemetry_parser.TelemetryFrame` received
                   from the Arduino telemetry stream.

        Note:
            Calling :meth:`record` before :meth:`start` is a no-op — the
            frame is silently dropped and a warning is logged.
        """
        if not self._profile_name:
            log.warning("record() called before start() — frame dropped")
            return

        # --- Grow buffers if full --------------------------------------------
        if self._count >= self._capacity:
            self._grow()

        # --- Write numeric fields into pre-allocated slots -------------------
        idx: int = self._count
        self._bufs["timestamp_ms"][idx]       = float(frame.timestamp_ms)
        self._bufs["pos_mm"][idx]             = frame.pos_mm
        self._bufs["vel_actual_mm_s"][idx]    = frame.vel_actual_mm_s
        self._bufs["vel_commanded_mm_s"][idx] = frame.vel_commanded_mm_s
        self._bufs["accel_mm_s2"][idx]        = frame.accel_mm_s2

        # --- Append string fields to lists -----------------------------------
        self._states.append(frame.state)
        self._phases.append(frame.phase)

        self._count += 1

    def finish(self, save: bool = True) -> RecordedRun:
        """
        End the current run, optionally save a CSV file, and return a
        :class:`RecordedRun` snapshot.

        Trims the internal numpy arrays to the number of frames actually
        recorded (discards the unused pre-allocated tail), then optionally
        writes a CSV file to :attr:`log_dir`.

        Args:
            save: If ``True`` (default), write a CSV file under
                  :attr:`log_dir`.  Pass ``False`` to discard data without
                  writing (e.g. when the run was aborted before any frames
                  arrived).

        Returns:
            A :class:`RecordedRun` containing trimmed arrays and the CSV
            path (or ``None`` if ``save=False`` or the run had 0 frames).

        Raises:
            RuntimeError: If called when no run is in progress (i.e.
                :meth:`start` was never called or :meth:`finish` was already
                called for the current run).

        Note:
            After :meth:`finish` returns, :attr:`is_recording` is ``False``
            and :attr:`frame_count` is ``0``.  The returned
            :class:`RecordedRun` owns its own copies of the arrays.
        """
        # Guard against double-finish or finish-without-start.
        if not self._profile_name:
            raise RuntimeError(
                "DataRecorder.finish() called with no run in progress. "
                "Call start() first."
            )

        # --- Trim arrays to actual frame count --------------------------------
        n: int = self._count
        trimmed: dict[str, Float64Array] = {
            col: self._bufs[col][:n].copy() if col in self._bufs else np.zeros(0)
            for col in _NUMERIC_COLS
        }
        states_copy: list[str] = list(self._states)
        phases_copy: list[str] = list(self._phases)

        # --- Optionally write CSV --------------------------------------------
        csv_path: Path | None = None
        if save and n > 0:
            csv_path = self._write_csv(trimmed, states_copy, phases_copy)

        log.debug(
            "DataRecorder finished: %d frames, csv=%s",
            n,
            csv_path,
        )

        run: RecordedRun = RecordedRun(
            profile_name = self._profile_name,
            started_at   = self._started_at,
            frame_count  = n,
            arrays       = trimmed,
            states       = states_copy,
            phases        = phases_copy,
            csv_path     = csv_path,
        )

        # --- Reset internal state so the recorder is ready for the next run --
        self._profile_name = ""
        self._started_at   = ""
        self._count        = 0
        self._capacity     = 0
        self._bufs         = {}
        self._states       = []
        self._phases       = []

        return run

    def current_arrays(self) -> dict[str, Float64Array]:
        """
        Return a view of the currently recorded numeric data (live, not a copy).

        Each array contains only the frames recorded so far — equivalent to
        ``buf[:frame_count]``.  The live-plot widget can call this at any
        time without triggering a copy or a lock.

        Returns:
            Dict mapping each numeric column name to a 1-D ``float64``
            numpy array slice.  Returns an empty dict if no run is active.

        Warning:
            The returned arrays share memory with the recorder's internal
            buffers.  Do **not** store the arrays across calls — a subsequent
            :meth:`record` that triggers a buffer growth will invalidate the
            previous slices.
        """
        if not self._profile_name:
            return {}
        n: int = self._count
        return {col: self._bufs[col][:n] for col in _NUMERIC_COLS}

    # -------------------------------------------------------------------------
    # Private helpers
    # -------------------------------------------------------------------------

    def _grow(self) -> None:
        """
        Double-append :data:`_CHUNK_SIZE` zeros to each internal numpy buffer.

        Called automatically by :meth:`record` when the current capacity is
        exhausted.  Uses ``numpy.concatenate`` rather than in-place resize to
        avoid invalidating existing slices held by the live-plot widget.
        """
        extension: Float64Array = np.zeros(_CHUNK_SIZE, dtype=np.float64)
        col: str
        for col in _NUMERIC_COLS:
            self._bufs[col] = np.concatenate([self._bufs[col], extension])
        self._capacity += _CHUNK_SIZE
        log.debug("DataRecorder buffers grown to %d frames", self._capacity)

    def _write_csv(
        self,
        arrays:  dict[str, Float64Array],
        states:  list[str],
        phases:  list[str],
    ) -> Path:
        """
        Write recorded data to a CSV file in :attr:`log_dir`.

        Filename format::

            run_<profile_name>_<YYYYMMDD_HHMMSS>.csv

        The timestamp comes from :attr:`_started_at` (UTC), converted to a
        compact local representation for readability.

        Columns written in order::

            timestamp_ms, pos_mm, vel_actual_mm_s, vel_commanded_mm_s,
            accel_mm_s2, state, phase

        Args:
            arrays:  Dict of trimmed float64 numpy arrays.
            states:  List of state strings.
            phases:  List of phase strings.

        Returns:
            The :class:`~pathlib.Path` of the written CSV file.

        Raises:
            OSError: If the file cannot be created (permissions, disk full).
        """
        # Ensure log directory exists.
        self.log_dir.mkdir(parents=True, exist_ok=True)

        # Build a compact timestamp string from the run's started_at field.
        # started_at is ISO-8601 UTC; strip down to YYYYMMDD_HHMMSS.
        ts_compact: str = (
            self._started_at.replace("-", "").replace(":", "").replace("+", "")[:15]
        )

        filename: str = f"run_{self._profile_name}_{ts_compact}.csv"
        path: Path    = self.log_dir / filename

        n: int = len(states)

        # Write header then one row per frame using a plain text approach.
        # numpy.savetxt handles homogeneous arrays only — mixing floats and
        # strings requires manual row construction.
        with path.open("w", encoding="utf-8", newline="") as fh:
            # --- Header row --------------------------------------------------
            fh.write(",".join(_ALL_COLS) + "\n")

            # --- Data rows ---------------------------------------------------
            row_idx: int
            for row_idx in range(n):
                # Build the float fields as fixed-precision strings.
                numeric_fields: list[str] = [
                    f"{arrays['timestamp_ms'][row_idx]:.0f}",
                    f"{arrays['pos_mm'][row_idx]:.4f}",
                    f"{arrays['vel_actual_mm_s'][row_idx]:.4f}",
                    f"{arrays['vel_commanded_mm_s'][row_idx]:.4f}",
                    f"{arrays['accel_mm_s2'][row_idx]:.4f}",
                ]
                row: str = ",".join(numeric_fields + [states[row_idx], phases[row_idx]])
                fh.write(row + "\n")

        log.info("CSV saved → %s (%d frames)", path, n)
        return path


# ---------------------------------------------------------------------------
# Module-level helper
# ---------------------------------------------------------------------------

def _sanitise_filename(name: str) -> str:
    """
    Replace characters that are unsafe in filenames with underscores.

    Applies to the profile name portion of the CSV filename so that any
    user-supplied profile name (including those with spaces, slashes, or
    other special characters) can be embedded safely in a path.

    Args:
        name: Raw profile name string.

    Returns:
        A string with every character in :data:`_FILENAME_UNSAFE_CHARS`
        replaced by ``"_"``, with leading/trailing underscores stripped.
        Returns ``"unnamed"`` if the result is empty after sanitisation.

    Examples:
        >>> _sanitise_filename("slow dip / fast withdraw")
        'slow_dip___fast_withdraw'
        >>> _sanitise_filename("   ")
        'unnamed'
    """
    sanitised: str = name
    ch: str
    for ch in _FILENAME_UNSAFE_CHARS:
        sanitised = sanitised.replace(ch, "_")

    sanitised = sanitised.strip("_")
    return sanitised if sanitised else "unnamed"
