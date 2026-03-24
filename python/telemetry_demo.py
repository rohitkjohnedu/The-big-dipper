"""
telemetry_demo.py
=================

Standalone preview of TelemetryTab.

Without arguments — MockArduino at 5x speed (no hardware needed):
    uv run python telemetry_demo.py

With a real Arduino on COM4:
    uv run python telemetry_demo.py --port COM4

With a different port or baud:
    uv run python telemetry_demo.py --port COM3 --baud 115200

What you see:
  - Numeric readout strip (position, velocity, accel, state, phase, frames)
  - Three live plots (position / velocity / acceleration)
  - Telem rate selector and Set button
  - Auto-record checkbox (CSV saved to logs/ on each RUNNING→other transition)
  - Export PNG and Clear buttons

Hardware mode notes:
  - The app connects on startup and sends SET_TELEM_RATE 10.
  - Close the Arduino IDE Serial Monitor before running — it holds COM4 exclusively.
  - The Home, Run controls are not present in this demo; use the full main.py UI
    (Step 26) or hw_test.py for issuing motion commands.
  - Click Ctrl+C in the terminal or close the window to disconnect cleanly.
"""

import argparse
import queue
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel
from PyQt6.QtCore import Qt

sys.path.insert(0, ".")

from core.command_interface import CommandInterface
from core.profile import DipProfile
from ui.tabs.telemetry_tab import TelemetryTab

SPEED_MULT = 5.0   # only used in MockArduino mode


# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Live telemetry demo — MockArduino or real hardware"
    )
    parser.add_argument(
        "--port", default=None,
        help="Serial port for real hardware (e.g. COM4). Omit to use MockArduino."
    )
    parser.add_argument(
        "--baud", type=int, default=115_200,
        help="Baud rate (default 115200, must match SERIAL_BAUD_RATE in config.h)"
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Demo window
# ---------------------------------------------------------------------------

class TelemetryDemoWindow(QMainWindow):
    def __init__(self, port: str | None, baud: int) -> None:
        super().__init__()
        self._hw_mode = port is not None
        self._manager = None

        # --- Backend -------------------------------------------------------
        if self._hw_mode:
            from core.serial_manager import SerialManager
            self._manager = SerialManager(port=port, baud=baud)
            try:
                self._manager.start()
            except Exception as exc:
                from PyQt6.QtWidgets import QMessageBox
                QMessageBox.critical(
                    None, "Connection failed",
                    f"Could not open {port} at {baud} baud:\n{exc}\n\n"
                    "Check:\n"
                    "  • Port name is correct\n"
                    "  • Arduino IDE Serial Monitor is closed\n"
                    "  • Arduino is powered and connected"
                )
                sys.exit(1)
            self._ci = CommandInterface(self._manager)
            source_label = f"Hardware  {port}  {baud} baud"
        else:
            from tests.mock_arduino import MockArduino
            self._manager = MockArduino(
                speed_multiplier=SPEED_MULT, telem_hz_default=10
            )
            self._manager.start()
            self._ci = CommandInterface(self._manager)  # type: ignore[arg-type]
            source_label = f"MockArduino  {SPEED_MULT}× speed"

        self.setWindowTitle(f"Dip Coater — Telemetry Demo  [{source_label}]")
        self.resize(1000, 740)

        # --- TelemetryTab --------------------------------------------------
        self._tab = TelemetryTab(log_dir="logs")
        self._tab.set_command_interface(self._ci)
        self._tab.notify_profile_name("demo")

        # --- Hint label ----------------------------------------------------
        if self._hw_mode:
            hint_text = (
                f"Connected to {port}.  "
                "Home the rig and run a profile to see live data.  "
                "CSV auto-saved to logs/ after each RUNNING→READY transition."
            )
        else:
            hint_text = (
                f"MockArduino at {SPEED_MULT}× speed.  "
                "Auto-record ON — CSV saved to logs/ after each run."
            )
        hint = QLabel(hint_text)
        hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint.setStyleSheet("color: #888888; font-size: 9pt;")

        central = QWidget()
        layout  = QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)
        layout.addWidget(hint)
        layout.addWidget(self._tab, stretch=1)
        self.setCentralWidget(central)
        self.setStyleSheet("QMainWindow { background-color: #2b2b2b; }")

        # --- Telemetry poll timer (20 Hz) ----------------------------------
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(50)
        self._poll_timer.timeout.connect(self._poll_telem)
        self._poll_timer.start()

        # --- Demo sequence (MockArduino only) ------------------------------
        if not self._hw_mode:
            self._step = 0
            self._seq = QTimer(self)
            self._seq.setSingleShot(True)
            self._seq.timeout.connect(self._next_step)
            self._seq.start(500)
        else:
            # Hardware mode: request telemetry at 10 Hz immediately.
            try:
                self._ci.set_telem_rate(10)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Telemetry polling
    # ------------------------------------------------------------------

    def _poll_telem(self) -> None:
        while True:
            try:
                frame = self._manager.telem_queue.get_nowait()
                self._tab.update_frame(frame)
            except queue.Empty:
                break

    # ------------------------------------------------------------------
    # MockArduino demo sequence
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
                self._ci.run(DipProfile(
                    name="demo",
                    dip_speed_mm_s=8.0,
                    withdraw_speed_mm_s=8.0,
                    accel_mm_s2=30.0,
                    dip_depth_mm=20.0,
                    dwell_bottom_ms=500,
                    dwell_top_ms=200,
                    n_dips=1,
                ))
            except Exception:
                pass
            self._seq.start(int(10_000 / SPEED_MULT))
        elif step == 2:
            self._tab._plot.clear()
            self._seq.start(1000)
        self._step += 1

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        self._poll_timer.stop()
        if not self._hw_mode:
            self._seq.stop()
        if self._manager is not None:
            self._manager.stop()
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    args = _parse_args()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = TelemetryDemoWindow(port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec())
