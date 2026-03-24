"""
ui/tabs/control_tab.py
======================

Primary operator tab — connection, manual jogging, and profile execution.

Three collapsible groups:

* **Connection** — port / baud entry, Connect / Disconnect button.
  Emits :attr:`connect_requested` / :attr:`disconnect_requested` signals
  so ``MainWindow`` can open or close the serial port without the tab
  needing to own a ``SerialManager``.

* **Manual Control** — Home, Stop, Jog UP / Jog DOWN with a speed
  spinbox.  Button states mirror the Arduino state machine: Jog is only
  enabled in ``READY``, Stop only in ``RUNNING`` or ``PAUSED``, etc.

* **Run Profile** — ``QComboBox`` of loaded profiles, Run / Pause /
  Resume buttons.  ``Run`` spawns a :class:`_RunWorker` ``QThread`` so
  segment streaming does not block the UI event loop.

Public API (called by ``MainWindow``)::

    tab.set_command_interface(ci)   # pass None to indicate disconnected
    tab.update_state(state)         # called on every telemetry frame
    tab.set_profiles(profiles)      # populate the profile combo box
"""

from __future__ import annotations

import logging
from typing import Final, Optional

from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMessageBox, QPushButton, QSpinBox,
    QVBoxLayout, QWidget,
)

from core.command_interface import CommandInterface, CommandError
from core.profile import DipProfile

log: Final[logging.Logger] = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Background worker — runs CommandInterface.run() off the UI thread
# ---------------------------------------------------------------------------

class _RunWorker(QThread):
    """Execute ``CommandInterface.run(profile)`` in a background thread.

    Emits :attr:`finished` on success or :attr:`error` with a message
    string on failure.  The UI thread connects these signals to update
    button states and show error dialogs.
    """

    finished: pyqtSignal = pyqtSignal()
    error:    pyqtSignal = pyqtSignal(str)

    def __init__(self, ci: CommandInterface, profile: DipProfile) -> None:
        super().__init__()
        self._ci      = ci
        self._profile = profile

    def run(self) -> None:
        try:
            self._ci.run(self._profile)
            self.finished.emit()
        except (CommandError, NotImplementedError, ValueError, Exception) as exc:
            self.error.emit(str(exc))


# ---------------------------------------------------------------------------
# ControlTab
# ---------------------------------------------------------------------------

class ControlTab(QWidget):
    """
    Primary operator tab.

    Signals:
        connect_requested(port, baud): User clicked Connect.
        disconnect_requested():        User clicked Disconnect.
    """

    connect_requested:    pyqtSignal = pyqtSignal(str, int)
    disconnect_requested: pyqtSignal = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._ci:         Optional[CommandInterface] = None
        self._state:      str                        = "IDLE"
        self._profiles:   list[DipProfile]           = []
        self._run_worker: Optional[_RunWorker]       = None

        self._build_ui()
        self._update_buttons()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_command_interface(self, ci: Optional[CommandInterface]) -> None:
        """Called by MainWindow when the connection state changes.

        Args:
            ci: Live :class:`~core.command_interface.CommandInterface`, or
                ``None`` when disconnected.
        """
        self._ci = ci
        if ci is not None:
            self._btn_connect.setText("Disconnect")
        else:
            self._btn_connect.setText("Connect")
        self._update_buttons()

    def update_state(self, state: str) -> None:
        """Refresh button enable/disable logic from the latest Arduino state.

        Args:
            state: One of ``IDLE | HOMING | READY | RUNNING | PAUSED | ERROR``.
        """
        self._state = state
        self._update_buttons()

    def set_profiles(self, profiles: list[DipProfile]) -> None:
        """Populate the profile selector combo box.

        Args:
            profiles: Ordered list of :class:`~core.profile.DipProfile`
                      instances.  The currently selected index is preserved
                      when the list is refreshed if the profile name still
                      exists.
        """
        prev_name: str = self._combo_profile.currentText()
        self._profiles = profiles
        self._combo_profile.clear()
        for p in profiles:
            self._combo_profile.addItem(p.name)
        # Restore previous selection if still present.
        idx = self._combo_profile.findText(prev_name)
        if idx >= 0:
            self._combo_profile.setCurrentIndex(idx)
        self._update_buttons()

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.setSpacing(8)
        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_manual_group())
        layout.addWidget(self._build_run_group())
        layout.addStretch()

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("Connection")
        row = QHBoxLayout(grp)
        row.setSpacing(6)

        self._edit_port = QLineEdit("COM4")
        self._edit_port.setFixedWidth(72)
        self._edit_port.setToolTip("Serial port (e.g. COM4 on Windows, /dev/ttyUSB0 on Linux)")

        self._spin_baud = QSpinBox()
        self._spin_baud.setRange(9600, 2_000_000)
        self._spin_baud.setValue(115_200)
        self._spin_baud.setFixedWidth(90)
        self._spin_baud.setToolTip("Baud rate — must match SERIAL_BAUD_RATE in config.h (115200)")

        self._btn_connect = QPushButton("Connect")
        self._btn_connect.setFixedWidth(100)
        self._btn_connect.clicked.connect(self._on_connect_clicked)

        row.addWidget(QLabel("Port:"))
        row.addWidget(self._edit_port)
        row.addWidget(QLabel("Baud:"))
        row.addWidget(self._spin_baud)
        row.addWidget(self._btn_connect)
        row.addStretch()
        return grp

    def _build_manual_group(self) -> QGroupBox:
        grp = QGroupBox("Manual Control")
        layout = QVBoxLayout(grp)
        layout.setSpacing(6)

        # Home / Stop
        row_hs = QHBoxLayout()
        self._btn_home = QPushButton("Home")
        self._btn_home.setToolTip("Drive to bottom endstop and zero position (IDLE / READY / ERROR)")
        self._btn_stop = QPushButton("Stop")
        self._btn_stop.setToolTip("Decelerated stop — motor halts gracefully (RUNNING / PAUSED)")
        self._btn_home.clicked.connect(self._on_home)
        self._btn_stop.clicked.connect(self._on_stop)
        row_hs.addWidget(self._btn_home)
        row_hs.addWidget(self._btn_stop)
        row_hs.addStretch()

        # Jog
        row_jog = QHBoxLayout()
        self._btn_jog_up   = QPushButton("Jog UP")
        self._btn_jog_down = QPushButton("Jog DOWN")
        self._spin_jog_spd = QDoubleSpinBox()
        self._spin_jog_spd.setRange(0.1, 50.0)
        self._spin_jog_spd.setValue(5.0)
        self._spin_jog_spd.setSuffix(" mm/s")
        self._spin_jog_spd.setDecimals(1)
        self._spin_jog_spd.setFixedWidth(100)
        self._btn_jog_up.setToolTip("Jog carriage upward — runs until Stop or ESTOP (READY only)")
        self._btn_jog_down.setToolTip("Jog carriage downward — runs until Stop or ESTOP (READY only)")
        self._btn_jog_up.clicked.connect(self._on_jog_up)
        self._btn_jog_down.clicked.connect(self._on_jog_down)
        row_jog.addWidget(self._btn_jog_up)
        row_jog.addWidget(self._btn_jog_down)
        row_jog.addWidget(QLabel("Speed:"))
        row_jog.addWidget(self._spin_jog_spd)
        row_jog.addStretch()

        layout.addLayout(row_hs)
        layout.addLayout(row_jog)
        return grp

    def _build_run_group(self) -> QGroupBox:
        grp = QGroupBox("Run Profile")
        layout = QVBoxLayout(grp)
        layout.setSpacing(6)

        self._combo_profile = QComboBox()
        self._combo_profile.setMinimumWidth(240)
        self._combo_profile.setToolTip("Select a dip profile to execute")

        row_btns = QHBoxLayout()
        self._btn_run    = QPushButton("Run")
        self._btn_pause  = QPushButton("Pause")
        self._btn_resume = QPushButton("Resume")
        self._btn_run.setToolTip("Execute the selected profile (READY only)")
        self._btn_pause.setToolTip("Pause mid-profile — motor decelerates to rest (RUNNING only)")
        self._btn_resume.setToolTip("Resume paused profile (PAUSED only)")
        self._btn_run.clicked.connect(self._on_run)
        self._btn_pause.clicked.connect(self._on_pause)
        self._btn_resume.clicked.connect(self._on_resume)
        row_btns.addWidget(self._btn_run)
        row_btns.addWidget(self._btn_pause)
        row_btns.addWidget(self._btn_resume)
        row_btns.addStretch()

        layout.addWidget(self._combo_profile)
        layout.addLayout(row_btns)
        return grp

    # ------------------------------------------------------------------
    # Private — button enable / disable
    # ------------------------------------------------------------------

    def _update_buttons(self) -> None:
        connected = self._ci is not None
        s         = self._state

        self._edit_port.setEnabled(not connected)
        self._spin_baud.setEnabled(not connected)

        self._btn_home.setEnabled(connected and s in ("IDLE", "READY", "ERROR"))
        self._btn_stop.setEnabled(connected and s in ("RUNNING", "PAUSED"))

        self._btn_jog_up.setEnabled(connected   and s == "READY")
        self._btn_jog_down.setEnabled(connected and s == "READY")

        has_profiles = len(self._profiles) > 0
        self._btn_run.setEnabled(connected    and s == "READY" and has_profiles)
        self._btn_pause.setEnabled(connected  and s == "RUNNING")
        self._btn_resume.setEnabled(connected and s == "PAUSED")
        self._combo_profile.setEnabled(connected and s not in ("RUNNING", "HOMING"))

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_connect_clicked(self) -> None:
        if self._ci is None:
            self.connect_requested.emit(
                self._edit_port.text().strip(),
                self._spin_baud.value(),
            )
        else:
            self.disconnect_requested.emit()

    def _on_home(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.home()
        except CommandError as exc:
            self._show_error("Home failed", str(exc))

    def _on_stop(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.stop()
        except CommandError as exc:
            self._show_error("Stop failed", str(exc))

    def _on_jog_up(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.jog("UP", self._spin_jog_spd.value())
        except CommandError as exc:
            self._show_error("Jog UP failed", str(exc))

    def _on_jog_down(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.jog("DOWN", self._spin_jog_spd.value())
        except CommandError as exc:
            self._show_error("Jog DOWN failed", str(exc))

    def _on_run(self) -> None:
        if self._ci is None or self._combo_profile.currentIndex() < 0:
            return
        idx:     int        = self._combo_profile.currentIndex()
        profile: DipProfile = self._profiles[idx]
        log.info("Starting run: %s", profile.name)
        self._run_worker = _RunWorker(self._ci, profile)
        self._run_worker.error.connect(
            lambda msg: self._show_error("Run failed", msg)
        )
        self._run_worker.start()

    def _on_pause(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.pause()
        except CommandError as exc:
            self._show_error("Pause failed", str(exc))

    def _on_resume(self) -> None:
        if self._ci is None:
            return
        try:
            self._ci.resume()
        except CommandError as exc:
            self._show_error("Resume failed", str(exc))

    def _show_error(self, title: str, msg: str) -> None:
        log.error("%s: %s", title, msg)
        QMessageBox.warning(self, title, msg)
