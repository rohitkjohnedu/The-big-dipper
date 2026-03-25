"""
mock_demo.py
============
Quick interactive demo of MockArduino — prints TELEM frames and command
responses as they arrive.

Run:
    uv run python mock_demo.py
"""

import queue
import sys
import time

# Allow running from the python/ directory directly.
sys.path.insert(0, ".")

from tests.mock_arduino import MockArduino

TELEM_HZ:   int   = 10
SPEED_MULT: float = 5.0    # 5× speed — a 4s move completes in ~0.8s

mock: MockArduino = MockArduino(speed_multiplier=SPEED_MULT, telem_hz_default=TELEM_HZ)
mock.start()
print(f"MockArduino started  (speed_multiplier={SPEED_MULT}, telem={TELEM_HZ} Hz)\n")


def send(cmd: str, expect_ack: bool = True) -> None:
    print(f"  TX: {cmd}")
    mock.send_command(cmd)
    if expect_ack:
        try:
            resp: str = mock.response_queue.get(timeout=2.0)
            print(f"  RX: {resp}")
        except queue.Empty:
            print("  RX: (no response within 2 s)")
    else:
        print("  RX: (no ACK expected for intermediate segment)")


def drain_telem(seconds: float, label: str = "") -> None:
    """Print all TELEM frames that arrive over *seconds*."""
    if label:
        print(f"\n[{label}]")
    deadline: float = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            from core.telemetry_parser import TelemetryFrame
            f: TelemetryFrame = mock.telem_queue.get(timeout=0.1)
            print(
                f"  TELEM  t={f.timestamp_ms:6d} ms  "
                f"pos={f.pos_mm:7.2f} mm  "
                f"vel={f.vel_commanded_mm_s:6.2f} mm/s  "
                f"state={f.state:<8}  phase={f.phase}"
            )
        except queue.Empty:
            pass


# -----------------------------------------------------------------------
print("=== 1. Initial TELEM (IDLE) ===")
drain_telem(0.5)

# -----------------------------------------------------------------------
print("\n=== 2. HOME ===")
send("CMD HOME")
drain_telem(1.5, "waiting for HOMING -> READY")

# -----------------------------------------------------------------------
print("\n=== 3. RUN_PROFILE  (20 mm, 8 mm/s, 1 dip, 500 ms dwell) ===")
send("CMD RUN_PROFILE 8.0 8.0 30.0 20.0 500 0 1")
drain_telem(4.0, "profile running")

# -----------------------------------------------------------------------
print("\n=== 4. Segmented move  (3 segments: slow in, fast bulk, slow out) ===")
send("CMD BEGIN_SEGMENTED_MOVE 3")
send("CMD MOVE_SEG -5.0 3.0 20.0",  expect_ack=False)  # no ACK
send("CMD MOVE_SEG -15.0 8.0 30.0", expect_ack=False)  # no ACK
send("CMD MOVE_SEG 20.0 8.0 30.0")                     # -> ACK PROFILE_READY
send("CMD RUN_LOADED_MOVE")
drain_telem(4.0, "segmented move running")

# -----------------------------------------------------------------------
print("\n=== 5. ESTOP ===")
send("CMD ESTOP")
drain_telem(0.3)

mock.stop()
print("\nMockArduino stopped.")
