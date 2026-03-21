"""
hw_test.py
==========

Hardware connectivity and velocity-profile test.
Run this with the Arduino connected on COM4.

Steps
-----
1. Connect and set telemetry rate.
2. Print 20 telemetry frames to verify parsing.
3. Optionally home the motor.
4. Optionally run a trapezoidal dip profile and stream live telemetry.

Run with::

    uv run python hw_test.py
"""

import queue
import sys
import time

from core.command_interface import CommandInterface, CommandError
from core.data_recorder import DataRecorder, RecordedRun
from core.profile import DipProfile
from core.serial_manager import SerialManager
from core.telemetry_parser import TelemetryFrame

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

PORT:     str = "COM4"
BAUD:     int = 115200
TELEM_HZ: int = 10

# ---------------------------------------------------------------------------
# Test profiles
# ---------------------------------------------------------------------------

# Conservative trapezoidal profile — 1 dip, slow speeds, shallow depth.
# Edit these values before running if your travel range differs.
TEST_PROFILE: DipProfile = DipProfile(
    name                = "hw_test",
    dip_speed_mm_s      = 5.0,
    withdraw_speed_mm_s = 8.0,
    accel_mm_s2         = 30.0,
    dip_depth_mm        = 20.0,
    dwell_bottom_ms     = 1000,
    dwell_top_ms        = 500,
    n_dips              = 1,
    notes               = "Hardware connectivity test — shallow, slow, single dip",
)

# Multi-speed segmented profile.
# Demonstrates variable speed during descent and withdraw:
#   1. Slow entry (first 5 mm) — minimise surface disturbance on entry
#   2. Fast bulk descent (remaining 15 mm)
#   3. Soak dwell at bottom (2 s)
#   4. Slow drainage withdraw (5 mm) — let excess coating drain off
#   5. Drainage pause (1 s)
#   6. Fast full withdraw (remaining 15 mm) back to home
#
# All distances are signed: negative = down, positive = up.
# Total travel: 20 mm down, 20 mm up — same depth as TEST_PROFILE.
SEGMENTED_PROFILE: DipProfile = DipProfile(
    name                 = "hw_test_segmented",
    dip_speed_mm_s       = 5.0,   # unused by segmented dispatch, kept for bookkeeping
    withdraw_speed_mm_s  = 8.0,
    accel_mm_s2          = 30.0,
    dip_depth_mm         = 20.0,
    dwell_bottom_ms      = 2000,
    dwell_top_ms         = 500,
    n_dips               = 1,
    notes                = "Multi-speed segmented test profile",
    velocity_profile_type = "segmented",
    velocity_profile_data = {
        "segments": [
            # ---- Descent -------------------------------------------------
            {
                "type":        "move",
                "distance_mm": -5.0,
                "speed_mm_s":  3.0,
                "accel_mm_s2": 20.0,
                "comment":     "slow entry — minimise surface disturbance",
            },
            {
                "type":        "move",
                "distance_mm": -15.0,
                "speed_mm_s":  10.0,
                "accel_mm_s2": 30.0,
                "comment":     "fast bulk descent to coating depth",
            },
            # ---- Soak at bottom -----------------------------------------
            {
                "type":        "dwell",
                "duration_ms": 2000,
                "comment":     "soak — hold substrate in coating solution",
            },
            # ---- Withdraw -----------------------------------------------
            {
                "type":        "move",
                "distance_mm": 5.0,
                "speed_mm_s":  2.0,
                "accel_mm_s2": 20.0,
                "comment":     "slow drainage withdraw — let excess coating drip",
            },
            {
                "type":        "dwell",
                "duration_ms": 1000,
                "comment":     "drainage pause — wait for drips to clear",
            },
            {
                "type":        "move",
                "distance_mm": 15.0,
                "speed_mm_s":  12.0,
                "accel_mm_s2": 30.0,
                "comment":     "fast full withdraw to home position",
            },
        ]
    },
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _print_frame(f: TelemetryFrame) -> None:
    """Print one telemetry frame to stdout in a fixed-width columnar format."""
    print(
        f"  t={f.timestamp_ms:>8} ms  "
        f"pos={f.pos_mm:>8.2f} mm  "
        f"vel_actual={f.vel_actual_mm_s:>7.2f} mm/s  "
        f"vel_cmd={f.vel_commanded_mm_s:>7.2f} mm/s  "
        f"state={f.state:<10}  phase={f.phase}"
    )


def _drain_telem(mgr: SerialManager, n: int, timeout_s: float = 10.0) -> None:
    """Print the next *n* telemetry frames, timing out after *timeout_s* s."""
    deadline: float = time.monotonic() + timeout_s
    received: int = 0
    while received < n:
        remaining: float = deadline - time.monotonic()
        if remaining <= 0:
            print(f"  [timeout — only received {received}/{n} frames]")
            return
        try:
            frame: TelemetryFrame = mgr.telem_queue.get(timeout=min(remaining, 1.0))
            _print_frame(frame)
            received += 1
        except queue.Empty:
            pass


def _flush_telem_queue(mgr: SerialManager) -> None:
    """Discard all frames already sitting in the telemetry queue.

    Call this immediately after sending a command so that the subsequent
    _wait_for_* functions only see frames produced AFTER the command was sent,
    not stale frames from a previous state.
    """
    discarded: int = 0
    while True:
        try:
            mgr.telem_queue.get_nowait()
            discarded += 1
        except queue.Empty:
            break
    if discarded:
        print(f"  [flushed {discarded} stale telemetry frame(s)]")


def _wait_for_state_transition(
    mgr: SerialManager,
    leave: str,
    arrive: str,
    timeout_leave_s: float = 5.0,
    timeout_arrive_s: float = 120.0,
    recorder: DataRecorder | None = None,
) -> bool:
    """Wait for the Arduino to leave one state and then arrive at another.

    This two-phase wait avoids the false-positive where the queue still holds
    stale frames from the *leave* state that were received before the command
    was sent.

    Phase 1 — wait up to *timeout_leave_s* for state != *leave*.
    Phase 2 — wait up to *timeout_arrive_s* for state == *arrive*.

    If *recorder* is provided, every frame seen in both phases is recorded.
    Returns True only if both phases succeed.
    """
    last_state: str = leave

    # ------------------------------------------------------------------
    # Phase 1: wait until the Arduino leaves the current state.
    # ------------------------------------------------------------------
    print(f"  Waiting for Arduino to leave {leave!r}...")
    deadline: float = time.monotonic() + timeout_leave_s
    left: bool = False

    while time.monotonic() < deadline:
        remaining: float = deadline - time.monotonic()
        try:
            frame: TelemetryFrame = mgr.telem_queue.get(timeout=min(remaining, 0.2))
            if recorder is not None:
                recorder.record(frame)
            if frame.state != last_state:
                print(f"  ↑ state: {last_state} → {frame.state}")
                last_state = frame.state
            if frame.state != leave:
                left = True
                break
        except queue.Empty:
            pass

    if not left:
        print(f"  [timeout — Arduino never left {leave!r}]")
        return False

    # ------------------------------------------------------------------
    # Phase 2: wait until the Arduino reaches the target state.
    # ------------------------------------------------------------------
    print(f"  Waiting for {arrive!r}...")
    deadline = time.monotonic() + timeout_arrive_s

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            frame = mgr.telem_queue.get(timeout=min(remaining, 1.0))
            if recorder is not None:
                recorder.record(frame)
            _print_frame(frame)
            if frame.state != last_state:
                print(f"  ↑ state: {last_state} → {frame.state}")
                last_state = frame.state
            if frame.state == arrive:
                return True
        except queue.Empty:
            pass

    print(f"  [timeout — last state={last_state!r}, expected={arrive!r}]")
    return False


def _print_run_summary(run: RecordedRun) -> None:
    """Print a brief statistical summary of a completed RecordedRun."""
    import numpy as np

    pos:     object = run.arrays.get("pos_mm")
    vel_act: object = run.arrays.get("vel_actual_mm_s")
    vel_cmd: object = run.arrays.get("vel_commanded_mm_s")

    print(f"\n  Frames recorded : {run.frame_count}")
    print(f"  Profile name    : {run.profile_name}")
    if pos is not None and hasattr(pos, "min"):
        import numpy.typing as npt
        import numpy as np_inner
        pos_arr: npt.NDArray[np.float64] = pos  # type: ignore[assignment]
        print(f"  Position range  : {float(pos_arr.min()):.2f} – {float(pos_arr.max()):.2f} mm")
    if vel_act is not None and hasattr(vel_act, "max"):
        va_arr: npt.NDArray[np.float64] = vel_act  # type: ignore[assignment]
        print(f"  Max actual vel  : {float(va_arr.max()):.2f} mm/s")
    if vel_cmd is not None and hasattr(vel_cmd, "max"):
        vc_arr: npt.NDArray[np.float64] = vel_cmd  # type: ignore[assignment]
        print(f"  Max cmd vel     : {float(vc_arr.max()):.2f} mm/s")
    if run.csv_path:
        print(f"  CSV saved to    : {run.csv_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 70)
    print(f"Dip coater hardware test  |  port={PORT}  baud={BAUD}")
    print("=" * 70)

    # ------------------------------------------------------------------
    # 1. Connect
    # ------------------------------------------------------------------
    print(f"\n[1] Connecting to {PORT}...")
    mgr: SerialManager = SerialManager(port=PORT, baud=BAUD)
    try:
        mgr.start()
    except Exception as exc:
        print(f"  ERROR: could not open {PORT}: {exc}")
        sys.exit(1)
    print("  Connected.  Waiting for Arduino boot (2 s)...")

    ci: CommandInterface = CommandInterface(mgr)

    # ------------------------------------------------------------------
    # 2. Telemetry rate + sanity frames
    # ------------------------------------------------------------------
    print(f"\n[2] Setting telemetry rate to {TELEM_HZ} Hz...")
    try:
        ci.set_telem_rate(TELEM_HZ)
        print("  ACK received.")
    except CommandError as exc:
        print(f"  WARNING: {exc}  (continuing anyway)")

    print("\n[3] Printing 20 telemetry frames...")
    _drain_telem(mgr, 20)

    # ------------------------------------------------------------------
    # 3. Optional home
    # ------------------------------------------------------------------
    print()
    if input("[4] Send CMD HOME? (y/N): ").strip().lower() == "y":
        print("  Sending HOME...")
        try:
            ci.home()
            print("  ACK received.")
            _flush_telem_queue(mgr)
            if not _wait_for_state_transition(
                mgr,
                leave="READY",
                arrive="READY",
                timeout_leave_s=5.0,
                timeout_arrive_s=60.0,
            ):
                print("  Homing did not complete — aborting.")
                mgr.stop()
                sys.exit(1)
            print("  Homing complete.\n")
        except CommandError as exc:
            print(f"  ERROR during home: {exc}")
            mgr.stop()
            sys.exit(1)
    else:
        print("  Skipping home.")

    # ------------------------------------------------------------------
    # 4. Optional profile run
    # ------------------------------------------------------------------
    print()
    print("  Test profile parameters:")
    print(f"    dip_speed       = {TEST_PROFILE.dip_speed_mm_s} mm/s")
    print(f"    withdraw_speed  = {TEST_PROFILE.withdraw_speed_mm_s} mm/s")
    print(f"    accel           = {TEST_PROFILE.accel_mm_s2} mm/s²")
    print(f"    dip_depth       = {TEST_PROFILE.dip_depth_mm} mm")
    print(f"    dwell_bottom    = {TEST_PROFILE.dwell_bottom_ms} ms")
    print(f"    dwell_top       = {TEST_PROFILE.dwell_top_ms} ms")
    print(f"    n_dips          = {TEST_PROFILE.n_dips}")
    print()

    if input("[5] Run trapezoidal profile? (y/N): ").strip().lower() == "y":
        # Set up recorder so we get a CSV and a summary.
        recorder: DataRecorder = DataRecorder(log_dir="logs")
        recorder.start(TEST_PROFILE.name)

        print("  Sending RUN_PROFILE...")
        try:
            ci.run(TEST_PROFILE)
            print("  ACK received.  Running — streaming live telemetry...\n")
        except CommandError as exc:
            print(f"  ERROR: {exc}")
            mgr.stop()
            sys.exit(1)

        _flush_telem_queue(mgr)

        # Stream telemetry: wait for READY→RUNNING, then RUNNING→READY.
        if _wait_for_state_transition(
            mgr,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=recorder,
        ):
            print("\n  Run complete.")
        else:
            print("\n  Run timed out or ended in an unexpected state.")

        # Finalise and print summary.
        run: RecordedRun = recorder.finish()
        _print_run_summary(run)

    else:
        print("  Skipping trapezoidal run.")

    # ------------------------------------------------------------------
    # 6. Optional segmented profile run
    # ------------------------------------------------------------------
    print()
    print("  Segmented profile — 6 steps, variable speed descent and withdraw:")
    for i, seg in enumerate(SEGMENTED_PROFILE.velocity_profile_data["segments"]):
        seg_type: str = seg["type"]
        comment:  str = seg.get("comment", "")
        if seg_type == "move":
            direction: str = "↓" if float(seg["distance_mm"]) < 0 else "↑"
            print(
                f"    [{i+1}] MOVE  {direction} {abs(float(seg['distance_mm'])):>5.1f} mm "
                f"@ {seg['speed_mm_s']} mm/s  accel={seg.get('accel_mm_s2', '?')} mm/s²"
                f"  — {comment}"
            )
        else:
            print(f"    [{i+1}] DWELL   {seg['duration_ms']} ms  — {comment}")
    print()

    if input("[6] Run segmented profile? (y/N): ").strip().lower() == "y":
        seg_recorder: DataRecorder = DataRecorder(log_dir="logs")
        seg_recorder.start(SEGMENTED_PROFILE.name)

        print("  Streaming segments to Arduino...")
        try:
            ci.run(SEGMENTED_PROFILE)
            print("  ACK RUN_LOADED_MOVE received.  Running...\n")
        except CommandError as exc:
            print(f"  ERROR: {exc}")
            mgr.stop()
            sys.exit(1)

        _flush_telem_queue(mgr)

        if _wait_for_state_transition(
            mgr,
            leave="READY",
            arrive="READY",
            timeout_leave_s=5.0,
            timeout_arrive_s=120.0,
            recorder=seg_recorder,
        ):
            print("\n  Segmented run complete.")
        else:
            print("\n  Segmented run timed out or ended in an unexpected state.")

        seg_run: RecordedRun = seg_recorder.finish()
        _print_run_summary(seg_run)

    else:
        print("  Skipping segmented run.")

    # ------------------------------------------------------------------
    # Done
    # ------------------------------------------------------------------
    print("\n[6] Disconnecting...")
    mgr.stop()
    print("  Done.")


if __name__ == "__main__":
    main()
