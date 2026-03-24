"""
tabs_demo.py
============

Standalone preview of all UI tabs before main_window.py is assembled.

Runs entirely with MockArduino — no hardware needed.

    uv run python tabs_demo.py

What you see:
  Tab 0 — ControlTab         (connection, home, jog, run/pause/stop)
  Tab 1 — SerialMonitorTab   (TX/RX log)
  Tab 2 — TelemetryTab       (live plots, Home button, Window/Full History)
  Tab 3 — ProfileManagerTab  (profile list, New/Duplicate/Rename/Delete/Load)
  Tab 4 — ProfileEditorTab   (stub — "not yet implemented")

MockArduino runs at 5× speed and auto-sequences HOME → RUN_PROFILE in a loop
so the telemetry plots show live data immediately.
"""

import queue
import sys
from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow,
    QTabWidget, QVBoxLayout, QWidget,
)
from PyQt6.QtCore import Qt

sys.path.insert(0, ".")

from core.command_interface import CommandInterface
from core.profile import DipProfile
from ui.tabs.control_tab import ControlTab
from ui.tabs.serial_monitor_tab import SerialMonitorTab
from ui.tabs.telemetry_tab import TelemetryTab
from ui.tabs.profile_manager_tab import ProfileManagerTab
from ui.tabs.profile_editor_tab import ProfileEditorTab
from ui.widgets.estop_button import EstopButton
from ui.widgets.status_bar import StatusBar
from tests.mock_arduino import MockArduino

SPEED_MULT   = 5.0
PROFILES_DIR = Path("profiles")
LOGS_DIR     = Path("logs")

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


class TabsDemoWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dip Coater — Tabs Demo  [MockArduino 5× speed]")
        self.resize(1100, 780)

        # --- Backend -------------------------------------------------------
        self._mock = MockArduino(speed_multiplier=SPEED_MULT, telem_hz_default=10)
        self._mock.start()
        self._ci = CommandInterface(self._mock)  # type: ignore[arg-type]

        # --- Tabs ----------------------------------------------------------
        self._tab_control  = ControlTab()
        self._tab_serial   = SerialMonitorTab()
        self._tab_telem    = TelemetryTab(log_dir=LOGS_DIR)
        self._tab_profiles = ProfileManagerTab(profile_dir=PROFILES_DIR)
        self._tab_editor   = ProfileEditorTab()

        # Wire up the tabs that need a CommandInterface
        self._tab_control.set_command_interface(self._ci)
        self._tab_serial.set_manager(self._mock)       # type: ignore[arg-type]
        self._tab_telem.set_command_interface(self._ci)
        self._tab_telem.notify_profile_name("demo")

        # Forward profile selection from manager → control tab
        self._tab_profiles.profile_selected.connect(
            lambda p: self._tab_control.set_profiles([p])
        )

        tabs = QTabWidget()
        tabs.addTab(self._tab_control,  "Control")
        tabs.addTab(self._tab_serial,   "Serial Monitor")
        tabs.addTab(self._tab_telem,    "Telemetry")
        tabs.addTab(self._tab_profiles, "Profiles")
        tabs.addTab(self._tab_editor,   "Profile Editor")

        # --- Status bar widget (above Qt status bar) -----------------------
        self._status = StatusBar()

        # --- ESTOP ---------------------------------------------------------
        self._estop = EstopButton(command_interface=self._ci)
        self._estop.setFixedHeight(48)

        # --- Layout --------------------------------------------------------
        top_row = QWidget()
        top_layout = QHBoxLayout(top_row)
        top_layout.setContentsMargins(4, 4, 4, 0)
        top_layout.addWidget(self._estop)
        top_layout.addStretch()

        hint = QLabel(
            "MockArduino at 5× speed  —  telemetry auto-starts after HOME completes"
        )
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color: #888888; font-size: 8pt;")

        central = QWidget()
        layout  = QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)
        layout.addWidget(top_row)
        layout.addWidget(hint)
        layout.addWidget(tabs, stretch=1)
        layout.addWidget(self._status)
        self.setCentralWidget(central)
        self.setStyleSheet("QMainWindow { background-color: #2b2b2b; }")

        # --- 20 Hz telemetry poll timer ------------------------------------
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(50)
        self._poll_timer.timeout.connect(self._poll_telem)
        self._poll_timer.start()

        # --- Demo sequence timer -------------------------------------------
        self._step = 0
        self._seq  = QTimer(self)
        self._seq.setSingleShot(True)
        self._seq.timeout.connect(self._next_step)
        self._seq.start(500)

    # ------------------------------------------------------------------
    # Telemetry polling
    # ------------------------------------------------------------------

    def _poll_telem(self) -> None:
        while True:
            try:
                frame = self._mock.telem_queue.get_nowait()
                self._tab_telem.update_frame(frame)
                self._status.update_frame(frame)
                self._tab_control.update_state(frame.state)
            except queue.Empty:
                break

    # ------------------------------------------------------------------
    # Demo sequence: HOME → RUN_PROFILE → repeat
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        self._poll_timer.stop()
        self._seq.stop()
        self._mock.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = TabsDemoWindow()
    win.show()
    sys.exit(app.exec())
