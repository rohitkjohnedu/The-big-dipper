"""
ui/main_window.py
=================

Top-level application window — assembles all tabs and owns the hardware
connection lifecycle.

Layout
------
::

    ┌──────────────────────────────────────────────────────────────┐
    │  [EMERGENCY STOP]                                            │  ← toolbar
    ├──────────────────────────────────────────────────────────────┤
    │  Control │ Serial Monitor │ Telemetry │ Profiles │ Editor   │  ← tabs
    │                                                              │
    │                    (tab content)                             │
    │                                                              │
    ├──────────────────────────────────────────────────────────────┤
    │  ● COM4   READY   NONE   00:00.0                            │  ← status bar
    └──────────────────────────────────────────────────────────────┘

Connection lifecycle
--------------------
``MainWindow`` owns the ``SerialManager``.

* **Connect** — ``ControlTab`` emits ``connect_requested(port, baud)``.
  ``MainWindow`` creates a ``SerialManager``, starts it, builds a
  ``CommandInterface``, and broadcasts it to every tab.

* **Disconnect** — ``ControlTab`` emits ``disconnect_requested``.
  ``MainWindow`` stops the manager, sets ``_manager`` and ``_ci`` to
  ``None``, and broadcasts ``None`` to every tab so they disable their
  hardware controls.

* **closeEvent** — tears down any active connection before the window
  closes.

Telemetry fan-out
-----------------
A single 20 Hz ``QTimer`` drains ``manager.telem_queue`` and delivers each
:class:`~core.telemetry_parser.TelemetryFrame` to:

* ``TelemetryTab.update_frame``
* ``StatusBar.update_frame``
* ``ControlTab.update_state``

Public API (used by ``main.py``)::

    win = MainWindow(profile_dir="profiles", log_dir="logs")
    win.show()
"""

from __future__ import annotations

import logging
import queue
from pathlib import Path
from typing import Final, Optional

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QCloseEvent
from PyQt6.QtWidgets import (
    QApplication, QHBoxLayout, QMainWindow, QMessageBox,
    QPushButton, QTabWidget, QVBoxLayout, QWidget,
)

from core.command_interface import CommandInterface
from core.profile import DipProfile, list_profiles
from core.serial_manager import SerialManager
from ui.tabs.control_tab import ControlTab
from ui.tabs.profile_editor_tab import ProfileEditorTab
from ui.tabs.profile_manager_tab import ProfileManagerTab
from ui.tabs.serial_monitor_tab import SerialMonitorTab
from ui.tabs.telemetry_tab import TelemetryTab
from ui.widgets.estop_button import EstopButton
from ui.widgets.status_bar import StatusBar

log: Final[logging.Logger] = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """
    Top-level application window.

    Args:
        profile_dir: Path to the ``profiles/`` directory.  Created if absent.
        log_dir:     Path to the ``logs/`` directory for CSV files.
        parent:      Optional parent widget.
    """

    def __init__(
        self,
        profile_dir: str | Path = "profiles",
        log_dir:     str | Path = "logs",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        self._profile_dir = Path(profile_dir)
        self._log_dir     = Path(log_dir)
        self._manager:    Optional[SerialManager]    = None
        self._ci:         Optional[CommandInterface] = None

        self._build_ui()
        self._load_initial_profiles()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_tab_control(self)  -> ControlTab:       return self._tab_control
    def get_tab_serial(self)   -> SerialMonitorTab:  return self._tab_serial
    def get_tab_telem(self)    -> TelemetryTab:      return self._tab_telem
    def get_tab_profiles(self) -> ProfileManagerTab: return self._tab_profiles
    def get_tab_editor(self)   -> ProfileEditorTab:  return self._tab_editor

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        self.setWindowTitle("Dip Coater Control")
        self.resize(1100, 800)
        self._dark_mode: bool = True
        self._apply_theme()

        # --- Tabs ----------------------------------------------------------
        self._tab_control:  ControlTab       = ControlTab()
        self._tab_serial:   SerialMonitorTab = SerialMonitorTab()
        self._tab_telem:    TelemetryTab     = TelemetryTab(log_dir=self._log_dir)
        self._tab_profiles: ProfileManagerTab = ProfileManagerTab(profile_dir=self._profile_dir)
        self._tab_editor:   ProfileEditorTab = ProfileEditorTab()

        self._tabs: QTabWidget = QTabWidget()
        self._tabs.addTab(self._tab_control,  "Control")
        self._tabs.addTab(self._tab_serial,   "Serial Monitor")
        self._tabs.addTab(self._tab_telem,    "Telemetry")
        self._tabs.addTab(self._tab_profiles, "Profiles")
        self._tabs.addTab(self._tab_editor,   "Profile Editor")

        # --- ESTOP ---------------------------------------------------------
        self._estop: EstopButton = EstopButton(command_interface=None)   # updated on connect
        self._estop.setFixedHeight(52)

        # --- Status bar widget (inside central widget, above Qt status bar) -
        self._status: StatusBar = StatusBar()

        # --- Toolbar row ---------------------------------------------------
        toolbar:        QWidget     = QWidget()
        toolbar_layout: QHBoxLayout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(4, 4, 4, 0)
        toolbar_layout.setSpacing(8)
        toolbar_layout.addWidget(self._estop)
        toolbar_layout.addStretch()

        self._btn_theme: QPushButton = QPushButton("☀  Light")
        self._btn_theme.setFixedWidth(90)
        self._btn_theme.setToolTip("Toggle light / dark theme")
        self._btn_theme.clicked.connect(self._on_toggle_theme)
        toolbar_layout.addWidget(self._btn_theme)

        # --- Central widget ------------------------------------------------
        central: QWidget     = QWidget()
        layout:  QVBoxLayout = QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 4)
        layout.setSpacing(4)
        layout.addWidget(toolbar)
        layout.addWidget(self._tabs, stretch=1)
        layout.addWidget(self._status)
        self.setCentralWidget(central)

        # --- Signal wiring -------------------------------------------------
        self._tab_control.connect_requested.connect(self._on_connect)
        self._tab_control.disconnect_requested.connect(self._on_disconnect)
        self._tab_profiles.profile_selected.connect(self._on_profile_selected)

        # --- 20 Hz telemetry poll timer ------------------------------------
        self._poll_timer: QTimer = QTimer(self)
        self._poll_timer.setInterval(50)
        self._poll_timer.timeout.connect(self._poll_telem)
        self._poll_timer.start()

    # ------------------------------------------------------------------
    # Private — connection lifecycle
    # ------------------------------------------------------------------

    def _on_connect(self, port: str, baud: int) -> None:
        """Open the serial port and wire up all tabs."""
        if self._manager is not None:
            self._teardown_connection()

        manager = SerialManager(port=port, baud=baud)
        try:
            manager.start()
        except Exception as exc:
            QMessageBox.critical(
                self, "Connection failed",
                f"Could not open {port} at {baud} baud:\n{exc}\n\n"
                "Check:\n"
                "  • Port name is correct\n"
                "  • Arduino IDE Serial Monitor is closed\n"
                "  • Arduino is powered and connected",
            )
            return

        self._manager = manager
        self._ci      = CommandInterface(self._manager)

        self._broadcast_ci(self._ci)
        self._status.set_connected(True, port)
        log.info("connected: %s @ %d baud", port, baud)

    def _on_disconnect(self) -> None:
        """Close the serial port and clear all tabs."""
        self._teardown_connection()
        log.info("disconnected")

    def _teardown_connection(self) -> None:
        """Stop the manager and push None to all tabs."""
        if self._manager is not None:
            self._manager.stop()
            self._manager = None
        self._ci = None
        self._broadcast_ci(None)
        self._status.set_connected(False)

    def _broadcast_ci(self, ci: Optional[CommandInterface]) -> None:
        """Push a CommandInterface (or None) to every tab that needs one."""
        self._tab_control.set_command_interface(ci)
        self._tab_telem.set_command_interface(ci)
        self._tab_serial.set_manager(self._manager)           # type: ignore[arg-type]
        self._estop.set_command_interface(ci)

    # ------------------------------------------------------------------
    # Private — profile selection
    # ------------------------------------------------------------------

    def _on_profile_selected(self, profile: DipProfile) -> None:
        """Forward a profile from ProfileManagerTab to ControlTab."""
        profiles = list_profiles(self._profile_dir)
        self._tab_control.set_profiles(profiles, select_name=profile.name)
        self._tab_telem.notify_profile_name(profile.name)
        # Switch to the Control tab so the operator can immediately run it.
        self._tabs.setCurrentWidget(self._tab_control)
        log.info("profile selected: %s", profile.name)

    def _load_initial_profiles(self) -> None:
        """Populate ControlTab with any profiles already on disk at startup."""
        self._profile_dir.mkdir(parents=True, exist_ok=True)
        profiles = list_profiles(self._profile_dir)
        if profiles:
            self._tab_control.set_profiles(profiles)

    # ------------------------------------------------------------------
    # Private — telemetry fan-out
    # ------------------------------------------------------------------

    def _poll_telem(self) -> None:
        if self._manager is None:
            return
        while True:
            try:
                frame = self._manager.telem_queue.get_nowait()
                self._tab_telem.update_frame(frame)
                self._status.update_frame(frame)
                self._tab_control.update_state(frame.state)
            except queue.Empty:
                break

    # ------------------------------------------------------------------
    # Theme
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Dark: near-black with blue accents.
    # Light: white/off-white with blue accents.
    # Jog pad buttons use setObjectName so generic QPushButton rules
    # do NOT affect them (ID selector #name has higher specificity).
    # ------------------------------------------------------------------

    _DARK_STYLESHEET: str = (
        "QWidget          { background-color: #0f1117; color: #e2e8f0; }"
        "QGroupBox         { border: 1px solid #30363d; border-radius: 6px;"
        "                    margin-top: 8px; color: #8b949e; }"
        "QGroupBox::title  { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        "QTabWidget::pane  { border: 1px solid #30363d; background: #161b22; }"
        "QTabBar::tab      { background: #161b22; color: #8b949e;"
        "                    padding: 6px 14px; border: 1px solid #30363d; }"
        "QTabBar::tab:selected { background: #1f6feb; color: #ffffff; border-color: #1f6feb; }"
        "QTabBar::tab:hover    { background: #21262d; color: #e2e8f0; }"
        "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {"
        "  background-color: #161b22; color: #e2e8f0;"
        "  border: 1px solid #30363d; border-radius: 4px; padding: 3px 6px; }"
        "QTableWidget      { background-color: #161b22; color: #e2e8f0;"
        "                    gridline-color: #30363d; alternate-background-color: #1a2028; }"
        "QHeaderView::section { background-color: #21262d; color: #8b949e;"
        "                       border: 1px solid #30363d; padding: 4px; }"
        "QScrollBar:vertical   { background: #161b22; width: 8px; border-radius: 4px; }"
        "QScrollBar::handle:vertical { background: #30363d; border-radius: 4px; }"
        # Generic buttons — NO border-radius so jog pad shapes are unaffected
        "QPushButton        { background-color: #21262d; color: #e2e8f0;"
        "                     border: 1px solid #30363d; padding: 5px 12px; }"
        "QPushButton:hover  { background-color: #30363d; border-color: #8b949e; }"
        "QPushButton:pressed{ background-color: #161b22; }"
        "QPushButton:disabled { background-color: #161b22; color: #484f58; }"
        "QCheckBox { color: #8b949e; spacing: 6px; }"
        "QLabel    { color: #8b949e; }"
        "QListWidget { background-color: #161b22; color: #e2e8f0; border: 1px solid #30363d; }"
    )

    _LIGHT_STYLESHEET: str = (
        "QWidget          { background-color: #f6f8fa; color: #24292f; }"
        "QGroupBox         { border: 1px solid #d0d7de; border-radius: 6px;"
        "                    margin-top: 8px; color: #57606a; }"
        "QGroupBox::title  { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
        "QTabWidget::pane  { border: 1px solid #d0d7de; background: #ffffff; }"
        "QTabBar::tab      { background: #f6f8fa; color: #57606a;"
        "                    padding: 6px 14px; border: 1px solid #d0d7de; }"
        "QTabBar::tab:selected { background: #0969da; color: #ffffff; border-color: #0969da; }"
        "QTabBar::tab:hover    { background: #eaeef2; color: #24292f; }"
        "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit {"
        "  background-color: #ffffff; color: #24292f;"
        "  border: 1px solid #d0d7de; border-radius: 4px; padding: 3px 6px; }"
        "QTableWidget      { background-color: #ffffff; color: #24292f;"
        "                    gridline-color: #d0d7de; alternate-background-color: #f6f8fa; }"
        "QHeaderView::section { background-color: #f6f8fa; color: #57606a;"
        "                       border: 1px solid #d0d7de; padding: 4px; }"
        "QScrollBar:vertical   { background: #f6f8fa; width: 8px; border-radius: 4px; }"
        "QScrollBar::handle:vertical { background: #d0d7de; border-radius: 4px; }"
        # Generic buttons — NO border-radius so jog pad shapes are unaffected
        "QPushButton        { background-color: #f6f8fa; color: #24292f;"
        "                     border: 1px solid #d0d7de; padding: 5px 12px; }"
        "QPushButton:hover  { background-color: #eaeef2; border-color: #57606a; }"
        "QPushButton:pressed{ background-color: #d0d7de; }"
        "QPushButton:disabled { background-color: #f6f8fa; color: #8c959f; }"
        "QCheckBox { color: #57606a; spacing: 6px; }"
        "QLabel    { color: #57606a; }"
        "QListWidget { background-color: #ffffff; color: #24292f; border: 1px solid #d0d7de; }"
    )

    def _apply_theme(self) -> None:
        app = QApplication.instance()
        if self._dark_mode:
            app.setStyleSheet(self._DARK_STYLESHEET)
        else:
            app.setStyleSheet(self._LIGHT_STYLESHEET)

    def _on_toggle_theme(self) -> None:
        self._dark_mode = not self._dark_mode
        self._apply_theme()
        self._btn_theme.setText("☀  Light" if self._dark_mode else "🌙  Dark")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event: QCloseEvent) -> None:
        self._poll_timer.stop()
        self._teardown_connection()
        super().closeEvent(event)
