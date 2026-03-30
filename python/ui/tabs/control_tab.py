"""
ui/tabs/control_tab.py
======================

Primary operator tab — connection, manual jogging, and profile execution.

Three collapsible groups:

* **Connection** — port / baud entry, Connect / Disconnect button.
  Emits :attr:`connect_requested` / :attr:`disconnect_requested` signals
  so ``MainWindow`` can open or close the serial port without the tab
  needing to own a ``SerialManager``.

* **Manual Control** — Jog pad widget with arc-shaped UP / DOWN buttons,
  a circular Home button, and a Stop button.  Jog buttons are
  press-and-hold: the motor moves while the button is held and stops when
  it is released.  Speed and acceleration spinboxes sit below the pad.

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
    QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
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
        self._ci:      CommandInterface = ci
        self._profile: DipProfile       = profile

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

    def set_profiles(
        self,
        profiles:    list[DipProfile],
        select_name: str | None = None,
    ) -> None:
        """Populate the profile selector combo box.

        Args:
            profiles:    Ordered list of :class:`~core.profile.DipProfile`
                         instances.
            select_name: Name of the profile to select after populating.
                         If ``None``, the previously selected name is
                         preserved when still present in the new list.
        """
        prev_name: str = select_name if select_name is not None \
                         else self._combo_profile.currentText()
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
        """Stack the three group boxes (Connection, Manual Control, Run Profile) vertically."""
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.setSpacing(8)
        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_manual_group())
        layout.addWidget(self._build_run_group())
        layout.addStretch()

    def _build_connection_group(self) -> QGroupBox:
        """Build the port / baud-rate entry row and the Connect/Disconnect button."""
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
        """Build the jog pad, step-move buttons, and custom move panel side by side."""
        grp = QGroupBox("Manual Control")
        row = QHBoxLayout(grp)
        row.setSpacing(16)
        row.setAlignment(Qt.AlignmentFlag.AlignTop)

        # Left: existing jog pad + speed/accel params below it.
        left = QVBoxLayout()
        left.setSpacing(8)
        left.addWidget(self._build_jog_pad())
        left.addLayout(self._build_jog_params())
        left.addStretch()

        row.addLayout(left)
        row.addWidget(self._build_step_move_panel())
        row.addWidget(self._build_custom_move_panel())
        row.addStretch()
        return grp

    def _build_step_move_panel(self) -> QGroupBox:
        """Build the fixed-distance step-move button column (+10/+5/+1/-1/-5/-10 mm).

        Each button sends CMD MOVE using the jog speed and acceleration spinboxes.
        All six buttons are stored in ``self._step_btns`` for enable/disable.
        """
        grp = QGroupBox("Step Move")
        col = QVBoxLayout(grp)
        col.setSpacing(4)

        self._step_btns: list[QPushButton] = []
        steps: list[tuple[float, str]] = [
            ( 10.0, "+10 mm"),
            (  5.0,  "+5 mm"),
            (  1.0,  "+1 mm"),
            ( -1.0,  "-1 mm"),
            ( -5.0,  "-5 mm"),
            (-10.0, "-10 mm"),
        ]
        for dist, label in steps:
            btn = QPushButton(label)
            btn.setFixedWidth(90)
            btn.setToolTip(
                f"Move {label} using jog speed and acceleration  (READY only)"
            )
            btn.clicked.connect(
                lambda checked, d=dist: self._on_step_move(d)
            )
            self._step_btns.append(btn)
            col.addWidget(btn)

        col.addStretch()
        return grp

    def _build_custom_move_panel(self) -> QGroupBox:
        """Build the custom move panel with settable distance, speed, and acceleration.

        Sends CMD MOVE with the values from the three spinboxes when the Move
        button is clicked.  Only enabled in READY state.
        """
        grp = QGroupBox("Custom Move")
        form = QFormLayout(grp)
        form.setSpacing(6)

        self._spin_move_dist = QDoubleSpinBox()
        self._spin_move_dist.setRange(-900.0, 900.0)
        self._spin_move_dist.setValue(10.0)
        self._spin_move_dist.setSuffix(" mm")
        self._spin_move_dist.setDecimals(1)
        self._spin_move_dist.setFixedWidth(110)
        self._spin_move_dist.setToolTip(
            "Move distance — positive = up, negative = down"
        )

        self._spin_move_spd = QDoubleSpinBox()
        self._spin_move_spd.setRange(0.1, 50.0)
        self._spin_move_spd.setValue(5.0)
        self._spin_move_spd.setSuffix(" mm/s")
        self._spin_move_spd.setDecimals(1)
        self._spin_move_spd.setFixedWidth(110)
        self._spin_move_spd.setToolTip("Travel speed for this move")

        self._spin_move_accel = QDoubleSpinBox()
        self._spin_move_accel.setRange(1.0, 500.0)
        self._spin_move_accel.setValue(20.0)
        self._spin_move_accel.setSuffix(" mm/s²")
        self._spin_move_accel.setDecimals(1)
        self._spin_move_accel.setFixedWidth(110)
        self._spin_move_accel.setToolTip("Acceleration ramp for this move")

        self._btn_custom_move = QPushButton("Move")
        self._btn_custom_move.setToolTip(
            "Execute a move with the distance, speed, and acceleration above  (READY only)"
        )
        self._btn_custom_move.clicked.connect(self._on_custom_move)

        form.addRow("Distance:", self._spin_move_dist)
        form.addRow("Speed:",    self._spin_move_spd)
        form.addRow("Accel:",    self._spin_move_accel)
        form.addRow(self._btn_custom_move)
        return grp

    def _build_jog_pad(self) -> QWidget:
        """Circular jog pad: UP arc (top), HOME circle (centre), DOWN arc (bottom), STOP (right)."""
        pad = QWidget()

        # --- Buttons -------------------------------------------------------
        self._btn_jog_up = QPushButton("↑")
        self._btn_jog_up.setObjectName("jogUp")
        self._btn_jog_up.setToolTip(
            "Jog UP — click to start, click Stop to halt  (READY only)"
        )
        self._btn_jog_up.clicked.connect(self._on_jog_up_pressed)
        self._btn_jog_up.setStyleSheet(
            "QPushButton#jogUp {"
            "  border-top-left-radius: 52px;"
            "  border-top-right-radius: 52px;"
            "  border-bottom-left-radius: 6px;"
            "  border-bottom-right-radius: 6px;"
            "  min-width: 130px; min-height: 54px;"
            "  font-size: 20pt; font-weight: bold;"
            "  background-color: #1565c0; color: white;"
            "}"
            "QPushButton#jogUp:pressed { background-color: #0d47a1; }"
            "QPushButton#jogUp:disabled { background-color: #555; color: #888; }"
        )

        self._btn_home = QPushButton("⌂")
        self._btn_home.setObjectName("jogHome")
        self._btn_home.setToolTip(
            "Home — drive to top endstop and zero position  (IDLE / READY / ERROR)"
        )
        self._btn_home.clicked.connect(self._on_home)
        self._btn_home.setStyleSheet(
            "QPushButton#jogHome {"
            "  border-radius: 42px;"
            "  min-width: 84px; max-width: 84px;"
            "  min-height: 84px; max-height: 84px;"
            "  font-size: 22pt;"
            "  background-color: #2e7d32; color: white;"
            "}"
            "QPushButton#jogHome:pressed { background-color: #1b5e20; }"
            "QPushButton#jogHome:disabled { background-color: #555; color: #888; }"
        )

        self._btn_jog_down = QPushButton("↓")
        self._btn_jog_down.setObjectName("jogDown")
        self._btn_jog_down.setToolTip(
            "Jog DOWN — click to start, click Stop to halt  (READY only)"
        )
        self._btn_jog_down.clicked.connect(self._on_jog_down_pressed)
        self._btn_jog_down.setStyleSheet(
            "QPushButton#jogDown {"
            "  border-bottom-left-radius: 52px;"
            "  border-bottom-right-radius: 52px;"
            "  border-top-left-radius: 6px;"
            "  border-top-right-radius: 6px;"
            "  min-width: 130px; min-height: 54px;"
            "  font-size: 20pt; font-weight: bold;"
            "  background-color: #1565c0; color: white;"
            "}"
            "QPushButton#jogDown:pressed { background-color: #0d47a1; }"
            "QPushButton#jogDown:disabled { background-color: #555; color: #888; }"
        )

        self._btn_stop = QPushButton("Stop")
        self._btn_stop.setObjectName("jogStop")
        self._btn_stop.setToolTip(
            "Stop — decelerated halt  (RUNNING / PAUSED / READY)"
        )
        self._btn_stop.clicked.connect(self._on_stop)
        self._btn_stop.setStyleSheet(
            "QPushButton#jogStop {"
            "  border-radius: 10px;"
            "  min-width: 80px; max-width: 80px;"
            "  min-height: 80px; max-height: 80px;"
            "  font-size: 11pt; font-weight: bold;"
            "  background-color: #b71c1c; color: white;"
            "}"
            "QPushButton#jogStop:pressed { background-color: #7f0000; }"
            "QPushButton#jogStop:disabled { background-color: #555; color: #888; }"
        )

        # --- Layout --------------------------------------------------------
        #       [ ↑  UP  ↑ ]
        #   [⌂ HOME]  [Stop]
        #       [ ↓ DOWN ↓ ]
        layout = QVBoxLayout(pad)
        layout.setSpacing(4)
        layout.setContentsMargins(0, 0, 0, 0)

        row_up = QHBoxLayout()
        row_up.addStretch()
        row_up.addWidget(self._btn_jog_up)
        row_up.addStretch()

        row_mid = QHBoxLayout()
        row_mid.setSpacing(16)
        row_mid.addStretch()
        row_mid.addWidget(self._btn_home)
        row_mid.addStretch()
        row_mid.addWidget(self._btn_stop)
        row_mid.addStretch()

        row_down = QHBoxLayout()
        row_down.addStretch()
        row_down.addWidget(self._btn_jog_down)
        row_down.addStretch()

        layout.addLayout(row_up)
        layout.addLayout(row_mid)
        layout.addLayout(row_down)
        return pad

    def _build_jog_params(self) -> QHBoxLayout:
        """Speed and acceleration spinboxes shown below the jog pad."""
        row = QHBoxLayout()
        row.setSpacing(8)

        self._spin_jog_spd = QDoubleSpinBox()
        self._spin_jog_spd.setRange(0.1, 50.0)
        self._spin_jog_spd.setValue(5.0)
        self._spin_jog_spd.setSuffix(" mm/s")
        self._spin_jog_spd.setDecimals(1)
        self._spin_jog_spd.setFixedWidth(110)
        self._spin_jog_spd.setToolTip("Jog speed")

        self._spin_jog_accel = QDoubleSpinBox()
        self._spin_jog_accel.setRange(1.0, 500.0)
        self._spin_jog_accel.setValue(20.0)
        self._spin_jog_accel.setSuffix(" mm/s²")
        self._spin_jog_accel.setDecimals(1)
        self._spin_jog_accel.setFixedWidth(120)
        self._spin_jog_accel.setToolTip("Jog acceleration")

        row.addStretch()
        row.addWidget(QLabel("Speed:"))
        row.addWidget(self._spin_jog_spd)
        row.addSpacing(12)
        row.addWidget(QLabel("Accel:"))
        row.addWidget(self._spin_jog_accel)
        row.addStretch()
        return row

    def _build_run_group(self) -> QGroupBox:
        """Build the profile combo box and Run / Pause / Resume buttons inside a group box."""
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
        """Enable or disable every interactive control based on connection state and Arduino state.

        Called whenever :attr:`_ci` or :attr:`_state` changes.  The rules are:

        * Port / baud fields — editable only when disconnected.
        * Home — allowed from IDLE, READY, or ERROR (recovery).
        * Stop — allowed from RUNNING, PAUSED, or READY.
        * Jog — allowed only from READY.
        * Run — allowed only from READY and only when at least one profile is loaded.
        * Pause — allowed only from RUNNING.
        * Resume — allowed only from PAUSED.
        * Profile combo — locked during RUNNING and HOMING to prevent mid-move changes.
        """
        connected:     bool = self._ci is not None
        arduino_state: str  = self._state

        # Connection fields are read-only once a port is open.
        self._edit_port.setEnabled(not connected)
        self._spin_baud.setEnabled(not connected)

        # Motion commands — validity depends on current Arduino state.
        self._btn_home.setEnabled(
            connected and arduino_state in ("IDLE", "READY", "ERROR")
        )
        self._btn_stop.setEnabled(
            connected and arduino_state in ("RUNNING", "PAUSED", "READY")
        )

        # Jogging and fixed-distance moves are only safe when homed and stationary.
        jog_ok: bool = connected and arduino_state == "READY"
        self._btn_jog_up.setEnabled(jog_ok)
        self._btn_jog_down.setEnabled(jog_ok)
        for btn in self._step_btns:
            btn.setEnabled(jog_ok)
        self._btn_custom_move.setEnabled(jog_ok)

        # Profile execution buttons.
        has_profiles: bool = len(self._profiles) > 0
        self._btn_run.setEnabled(
            connected and arduino_state == "READY" and has_profiles
        )
        self._btn_pause.setEnabled(connected  and arduino_state == "RUNNING")
        self._btn_resume.setEnabled(connected and arduino_state == "PAUSED")

        # Lock the combo during motion so the operator cannot swap profiles mid-run.
        self._combo_profile.setEnabled(
            connected and arduino_state not in ("RUNNING", "HOMING")
        )

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_connect_clicked(self) -> None:
        """Emit the appropriate connection signal depending on the current state.

        If no command interface is set (disconnected), emits :attr:`connect_requested`
        with the port name and baud rate from the UI fields.  If already connected,
        emits :attr:`disconnect_requested`.
        """
        if self._ci is None:
            self.connect_requested.emit(
                self._edit_port.text().strip(),
                self._spin_baud.value(),
            )
        else:
            self.disconnect_requested.emit()

    def _on_home(self) -> None:
        """Send CMD HOME to drive the axis to the endstop and zero the encoder."""
        if self._ci is None:
            return
        try:
            self._ci.home()
        except CommandError as exc:
            self._show_error("Home failed", str(exc))

    def _on_stop(self) -> None:
        """Send CMD STOP for a controlled decelerated halt."""
        if self._ci is None:
            return
        try:
            self._ci.stop()
        except CommandError as exc:
            self._show_error("Stop failed", str(exc))

    def _on_jog_up_pressed(self) -> None:
        """Start continuous upward jog at the speed and acceleration set in the spinboxes."""
        if self._ci is None:
            return
        try:
            self._ci.jog_start(
                "UP",
                self._spin_jog_spd.value(),
                self._spin_jog_accel.value(),
            )
        except (CommandError, ValueError) as exc:
            log.warning("Jog UP failed: %s", exc)

    def _on_jog_down_pressed(self) -> None:
        """Start continuous downward jog at the speed and acceleration set in the spinboxes."""
        if self._ci is None:
            return
        try:
            self._ci.jog_start(
                "DOWN",
                self._spin_jog_spd.value(),
                self._spin_jog_accel.value(),
            )
        except (CommandError, ValueError) as exc:
            log.warning("Jog DOWN failed: %s", exc)

    def _on_step_move(self, distance_mm: float) -> None:
        """Send CMD MOVE for a fixed step distance using the jog speed and acceleration.

        Args:
            distance_mm: Signed displacement — positive = up, negative = down.
        """
        if self._ci is None:
            return
        try:
            self._ci.move_by_mm(
                distance_mm,
                self._spin_jog_spd.value(),
                self._spin_jog_accel.value(),
            )
        except CommandError as exc:
            self._show_error("Move failed", str(exc))

    def _on_custom_move(self) -> None:
        """Send CMD MOVE using the distance, speed, and acceleration from the custom move panel."""
        if self._ci is None:
            return
        try:
            self._ci.move_by_mm(
                self._spin_move_dist.value(),
                self._spin_move_spd.value(),
                self._spin_move_accel.value(),
            )
        except CommandError as exc:
            self._show_error("Move failed", str(exc))

    def _on_run(self) -> None:
        """Spawn a _RunWorker thread to execute the selected profile without blocking the UI.

        The worker emits ``error(str)`` on failure; that signal is wired to
        :meth:`_show_error` so the operator sees a dialog if something goes wrong.
        """
        if self._ci is None or self._combo_profile.currentIndex() < 0:
            return

        # Resolve the selected profile object from the combo index.
        idx:     int        = self._combo_profile.currentIndex()
        profile: DipProfile = self._profiles[idx]
        log.info("Starting run: %s", profile.name)

        # Run the blocking profile-streaming call in a background thread.
        self._run_worker = _RunWorker(self._ci, profile)
        self._run_worker.error.connect(
            lambda msg: self._show_error("Run failed", msg)
        )
        self._run_worker.start()

    def _on_pause(self) -> None:
        """Send CMD PAUSE to suspend the active profile at the current position."""
        if self._ci is None:
            return
        try:
            self._ci.pause()
        except CommandError as exc:
            self._show_error("Pause failed", str(exc))

    def _on_resume(self) -> None:
        """Send CMD RESUME to continue a paused profile from where it stopped."""
        if self._ci is None:
            return
        try:
            self._ci.resume()
        except CommandError as exc:
            self._show_error("Resume failed", str(exc))

    def _show_error(self, title: str, msg: str) -> None:
        """Log *msg* at ERROR level and display a warning dialog to the operator.

        Args:
            title: Dialog window title and log prefix.
            msg:   Human-readable error description.
        """
        log.error("%s: %s", title, msg)
        QMessageBox.warning(self, title, msg)
