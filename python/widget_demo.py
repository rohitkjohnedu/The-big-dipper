"""
widget_demo.py
==============

Quick visual preview of the three UI widgets wired to MockArduino.

Run:
    uv run python widget_demo.py

The window shows:
  - EstopButton (top)        -- click to send ESTOP
  - StatusBar   (middle)     -- live state / phase / elapsed time
  - LivePlot    (bottom)     -- scrolling position / velocity / accel

A QTimer polls MockArduino's telem_queue at 20 Hz and feeds frames to
the plot and status bar.  A second timer runs a short demo sequence:
  1. HOME
  2. RUN_PROFILE (20 mm, 8 mm/s, 1 dip, 500 ms dwell)
  3. Wait for READY, then repeat.
"""

import sys
import queue

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QCloseEvent
from PyQt6.QtWidgets import (
    QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget,
)

sys.path.insert(0, ".")

from core.command_interface import CommandInterface
from tests.mock_arduino import MockArduino
from ui.widgets.estop_button import EstopButton
from ui.widgets.live_plot import LivePlot
from ui.widgets.status_bar import StatusBar

SPEED_MULT = 5.0   # 5x speed so the demo runs quickly


class DemoWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dip Coater — Widget Demo")
        self.resize(900, 700)

        # --- Backend -------------------------------------------------------
        self._mock: MockArduino     = MockArduino(speed_multiplier=SPEED_MULT, telem_hz_default=10)
        self._mock.start()
        self._ci:   CommandInterface = CommandInterface(self._mock)  # type: ignore[arg-type]

        # --- Widgets -------------------------------------------------------
        self._estop:  EstopButton = EstopButton(command_interface=self._ci)
        self._status: StatusBar   = StatusBar(port="MockArduino")
        self._status.set_connected(True)
        self._plot:   LivePlot    = LivePlot(history_s=60.0, telem_hz=10)

        # Layout
        central: QWidget     = QWidget()
        layout:  QVBoxLayout = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        hint: QLabel = QLabel(
            "MockArduino running at 5x speed.  "
            "Click EMERGENCY STOP to halt at any time."
        )
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color: #aaaaaa; font-size: 9pt;")

        layout.addWidget(self._estop)
        layout.addWidget(self._status)
        layout.addWidget(hint)
        layout.addWidget(self._plot, stretch=1)

        self.setCentralWidget(central)
        self.setStyleSheet("QMainWindow { background-color: #2b2b2b; }")

        # --- Telemetry poll timer (20 Hz) ----------------------------------
        self._telem_timer: QTimer = QTimer(self)
        self._telem_timer.setInterval(50)
        self._telem_timer.timeout.connect(self._poll_telem)
        self._telem_timer.start()

        # --- Demo sequence timer -------------------------------------------
        self._seq_step:  int   = 0
        self._seq_timer: QTimer = QTimer(self)
        self._seq_timer.setSingleShot(True)
        self._seq_timer.timeout.connect(self._next_seq_step)
        self._seq_timer.start(500)   # first step after 0.5 s

    # ------------------------------------------------------------------

    def _poll_telem(self) -> None:
        """Drain telem_queue and forward frames to plot + status bar."""
        from core.telemetry_parser import TelemetryFrame
        while True:
            try:
                frame: TelemetryFrame = self._mock.telem_queue.get_nowait()
                self._plot.push_frame(frame)
                self._status.update_frame(frame)
            except queue.Empty:
                break

    def _next_seq_step(self) -> None:
        """Advance through a simple repeating HOME → RUN_PROFILE loop."""
        step: int = self._seq_step % 3

        if step == 0:
            # Home
            try:
                self._ci.home()
            except Exception:
                pass
            self._seq_timer.start(int(2000 / SPEED_MULT))

        elif step == 1:
            # Run a short profile
            try:
                self._ci.run(self._build_profile())
            except Exception:
                pass
            self._seq_timer.start(int(10_000 / SPEED_MULT))

        elif step == 2:
            # Brief pause at READY before repeating
            self._plot.clear()
            self._seq_timer.start(1000)

        self._seq_step += 1

    @staticmethod
    def _build_profile() -> "DipProfile":
        from core.profile import DipProfile
        return DipProfile(
            name                = "demo",
            dip_speed_mm_s      = 8.0,
            withdraw_speed_mm_s = 8.0,
            accel_mm_s2         = 30.0,
            dip_depth_mm        = 20.0,
            dwell_bottom_ms     = 500,
            dwell_top_ms        = 200,
            n_dips              = 1,
        )

    def closeEvent(self, event: QCloseEvent) -> None:
        self._telem_timer.stop()
        self._seq_timer.stop()
        self._mock.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app: QApplication = QApplication(sys.argv)
    app.setStyle("Fusion")
    win: DemoWindow = DemoWindow()
    win.show()
    sys.exit(app.exec())
