"""
tabs_demo.py
============

Preview MainWindow with MockArduino — no hardware needed.

Launches the real MainWindow (identical to what main.py will show) and
injects a MockArduino backend so all tabs are live without a physical rig.

    uv run python tabs_demo.py

The MockArduino runs at 5× speed and automatically sequences:
    HOME → RUN_PROFILE → (repeat)

so the Telemetry tab shows live plots immediately.  All other tabs are
fully interactive — create profiles, use the serial monitor, try the
Home button and Window/Full History toggle on the plots.

To test with real hardware instead:
    uv run python telemetry_demo.py --port COM4
    (full main.py with hardware will be available after Step 26)
"""

import queue
import sys
from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

sys.path.insert(0, ".")

from core.command_interface import CommandInterface
from core.profile import DipProfile
from tests.mock_arduino import MockArduino
from ui.main_window import MainWindow

SPEED_MULT = 5.0

_DEMO_PROFILE = DipProfile(
    name="demo",
    dip_speed_mm_s=8.0,
    withdraw_speed_mm_s=8.0,
    accel_mm_s2=30.0,
    dip_depth_mm=20.0,
    dwell_bottom_ms=500,
    dwell_top_ms=200,
    n_dips=1,
)


def _inject_mock(win: MainWindow) -> MockArduino:
    """Replace the real SerialManager with a MockArduino inside MainWindow."""
    mock = MockArduino(speed_multiplier=SPEED_MULT, telem_hz_default=10)
    mock.start()
    ci = CommandInterface(mock)   # type: ignore[arg-type]
    win._manager = mock           # type: ignore[assignment]
    win._ci      = ci
    win._broadcast_ci(ci)
    win._status.set_connected(True, "MockArduino")
    win._tab_control._btn_connect.setText("Disconnect")
    return mock


class _DemoRunner:
    """Drives the MockArduino demo sequence: HOME → RUN → repeat."""

    def __init__(self, win: MainWindow, mock: MockArduino) -> None:
        self._win  = win
        self._mock = mock
        self._ci   = win._ci
        self._step = 0
        self._seq  = QTimer()
        self._seq.setSingleShot(True)
        self._seq.timeout.connect(self._next_step)
        self._seq.start(500)

    def stop(self) -> None:
        self._seq.stop()

    def _next_step(self) -> None:
        step = self._step % 3
        if step == 0:
            try:
                self._ci.home()
            except Exception:
                pass
            self._seq.start(int(2000 / SPEED_MULT))
        elif step == 1:
            try:
                self._ci.run(_DEMO_PROFILE)
            except Exception:
                pass
            self._seq.start(int(10_000 / SPEED_MULT))
        elif step == 2:
            self._seq.start(1000)
        self._step += 1


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = MainWindow(
        profile_dir=Path("profiles"),
        log_dir=Path("logs"),
    )
    win.setWindowTitle("Dip Coater — Preview  [MockArduino 5× speed]")

    mock   = _inject_mock(win)
    runner = _DemoRunner(win, mock)

    # Clean shutdown
    original_close = win.closeEvent
    def _close(event):
        runner.stop()
        mock.stop()
        original_close(event)
    win.closeEvent = _close   # type: ignore[method-assign]

    win.show()
    sys.exit(app.exec())
