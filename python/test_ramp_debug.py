"""
test_ramp_debug.py
==================
Headless smoke-test for LivePlot's computed-accel + ramped-velocity logic.

Run from python/:
    uv run pytest test_ramp_debug.py -v -s

What it does
------------
Pushes 25 synthetic TelemetryFrames to a LivePlot instance:
  - Phase 1 (frames 0-9):  vel_actual ramps 0 → 30 mm/s (motor accelerating)
  - Phase 2 (frames 10-14): vel_actual holds 30 mm/s (cruise)
  - Phase 3 (frames 15-24): vel_actual ramps 30 → 0 mm/s (motor decelerating)

vel_commanded_mm_s is fixed at 30 (unsigned, as the firmware sends it).
Prints last_vel_cmd_ramped and last_accel_computed after every frame so you can
verify the ramp rises smoothly rather than jumping to 30 instantly.
"""
from __future__ import annotations

import pytest
from core.telemetry_parser import TelemetryFrame
from ui.widgets.live_plot import LivePlot


def _frame(ts_ms: int, vel_act: float, vel_cmd: float = 30.0) -> TelemetryFrame:
    return TelemetryFrame(
        timestamp_ms=ts_ms,
        pos_mm=0.0,
        vel_actual_mm_s=vel_act,
        vel_commanded_mm_s=vel_cmd,
        accel_mm_s2=0.0,
        state="RUNNING",
        phase="DESCENDING",
    )


@pytest.fixture
def plot(qtbot):
    p = LivePlot(history_s=60.0, telem_hz=10)
    qtbot.addWidget(p)
    return p


def test_commanded_tracks_target_directly(plot):
    """
    Commanded velocity should equal the noise-floored signed target directly.

    No synthetic ramp is applied: commanded shows the controller's intent
    (step to target), actual shows the motor's physical response.
    The accel subplot uses EMA-smoothed dv/dt and is verified separately.
    """
    TELEM_HZ = 10
    DT_MS    = 1000 // TELEM_HZ
    PEAK_VEL = 30.0
    N_RAMP   = 10
    N_CRUISE = 5
    N_DOWN   = 10

    frames: list[TelemetryFrame] = []
    ts = 1000

    frames.append(_frame(ts, 0.0))
    ts += DT_MS

    for i in range(N_RAMP):
        frames.append(_frame(ts, PEAK_VEL * (i + 1) / N_RAMP))
        ts += DT_MS

    for _ in range(N_CRUISE):
        frames.append(_frame(ts, PEAK_VEL))
        ts += DT_MS

    for i in range(N_DOWN):
        frames.append(_frame(ts, PEAK_VEL * (N_DOWN - i - 1) / N_DOWN))
        ts += DT_MS

    print("\n")
    print(f"{'Frame':>5}  {'vel_actual':>10}  {'vel_cmd_ramped':>14}  {'accel_ema':>10}")
    print("-" * 48)

    _VEL_NOISE = 0.5
    for i, frame in enumerate(frames):
        plot.push_frame(frame)
        ramped = plot.last_vel_cmd_ramped
        accel  = plot.last_accel_computed
        print(f"{i:>5}  {frame.vel_actual_mm_s:>10.2f}  {ramped:>14.3f}  {accel:>10.3f}")

        # Commanded should equal the noise-floored signed target
        if abs(frame.vel_actual_mm_s) < _VEL_NOISE:
            assert ramped == 0.0, f"Frame {i}: expected 0 when stationary, got {ramped}"
        else:
            import math
            expected = math.copysign(frame.vel_commanded_mm_s, frame.vel_actual_mm_s)
            assert abs(ramped - expected) < 0.01, (
                f"Frame {i}: expected {expected:.3f}, got {ramped:.3f}"
            )


def test_ramp_reaches_target(plot):
    """After enough cruise frames, ramped vel should converge to target."""
    ts = 1000
    # Jump instantly to full velocity (as if motor already at speed)
    for i in range(30):
        plot.push_frame(_frame(ts, 30.0))
        ts += 100

    assert abs(plot.last_vel_cmd_ramped - 30.0) < 1.0, (
        f"Ramped vel {plot.last_vel_cmd_ramped:.2f} never converged to 30 mm/s"
    )


def test_ramp_returns_to_zero(plot):
    """When motor stops, ramped vel should return to 0."""
    ts = 1000
    # First run at speed
    for _ in range(20):
        plot.push_frame(_frame(ts, 30.0))
        ts += 100
    # Then stop
    for _ in range(20):
        plot.push_frame(_frame(ts, 0.0))
        ts += 100

    assert abs(plot.last_vel_cmd_ramped) < 1.0, (
        f"Ramped vel {plot.last_vel_cmd_ramped:.2f} did not return to 0 after motor stop"
    )
