"""
ui/tabs/telemetry_tab.py
========================

Live telemetry display — numeric readouts, scrolling plots, and recording
controls.

Layout
------
::

    ┌─────────────────────────────────────────────────────────┐
    │  Pos  │  Vel Act  │  Vel Cmd  │  Accel  │ State │ Phase │  ← readout strip
    ├─────────────────────────────────────────────────────────┤
    │                                                         │
    │                    LivePlot                             │  ← fills remaining space
    │            (position / velocity / accel)                │
    │                                                         │
    ├─────────────────────────────────────────────────────────┤
    │ Telem Hz: [spin][Set]   Auto-record [✓]  [Rec●] [PNG] [Clear] │
    └─────────────────────────────────────────────────────────┘

Public API (called by ``MainWindow``)::

    tab.set_command_interface(ci)     # pass None when disconnected
    tab.update_frame(frame)           # call on every telemetry frame
    tab.notify_profile_name(name)     # call when operator selects a profile

Recording behaviour
-------------------
When **Auto-record** is checked (default on), recording starts automatically
when the Arduino state transitions to ``RUNNING`` and ends (saving CSV) when
it leaves ``RUNNING``.  The operator can also start/stop recording manually
via the ``Rec●`` toggle button regardless of the Arduino state.

The CSV is written to ``logs/`` relative to the working directory.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Final, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFileDialog, QGroupBox, QHBoxLayout,
    QLabel, QMessageBox, QPushButton, QSpinBox, QVBoxLayout, QWidget,
)

from core.command_interface import CommandInterface, CommandError
from core.data_recorder import DataRecorder
from core.telemetry_parser import TelemetryFrame
from ui.widgets.live_plot import LivePlot

log: Final[logging.Logger] = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _readout_label(title: str, width: int = 110) -> tuple[QLabel, QLabel]:
    """Return a (title_label, value_label) pair for the readout strip."""
    lbl_title: QLabel = QLabel(title)
    lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lbl_title.setStyleSheet(
        "QLabel { color: #888888; font-size: 8pt; }"
    )

    lbl_value: QLabel = QLabel("—")
    lbl_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
    lbl_value.setMinimumWidth(width)
    lbl_value.setStyleSheet(
        "QLabel {"
        "  color: #eeeeee;"
        "  font-size: 11pt;"
        "  font-weight: bold;"
        "  background-color: #2a2a2a;"
        "  border: 1px solid #444;"
        "  border-radius: 4px;"
        "  padding: 3px 6px;"
        "}"
    )
    return lbl_title, lbl_value


# ---------------------------------------------------------------------------
# TelemetryTab
# ---------------------------------------------------------------------------

class TelemetryTab(QWidget):
    """
    Live telemetry tab — numeric readouts, LivePlot, and DataRecorder controls.

    Args:
        log_dir: Directory for CSV files.  Passed straight to
                 :class:`~core.data_recorder.DataRecorder`.
        parent:  Optional parent widget.
    """

    def __init__(self, log_dir: str | Path = "logs", parent=None) -> None:
        super().__init__(parent)
        self._ci:           Optional[CommandInterface] = None
        self._profile_name: str                        = "unnamed"
        self._last_state:   str                        = ""

        self._recorder = DataRecorder(log_dir=log_dir)

        self._build_ui()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_command_interface(self, ci: Optional[CommandInterface]) -> None:
        """Called by MainWindow on connect / disconnect."""
        self._ci = ci
        self._btn_set_rate.setEnabled(ci is not None)

    def notify_profile_name(self, name: str) -> None:
        """Tell the tab which profile is selected so the CSV filename is correct."""
        self._profile_name = name or "unnamed"

    def update_frame(self, frame: TelemetryFrame) -> None:
        """
        Feed one telemetry frame to the readouts, plot, and recorder.

        Must be called from the Qt main thread.
        """
        self._plot.push_frame(frame)
        self._handle_recording(frame)
        # Record after _handle_recording so that the first RUNNING frame is
        # captured (recording is started inside _handle_recording on the
        # transition into RUNNING, so is_recording is True by the time we arrive here).
        if self._recorder.is_recording:
            self._recorder.record(frame)
        # Update readouts last so frame_count label reflects the just-recorded frame.
        self._update_readouts(frame)
        self._last_state = frame.state

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        """Assemble the tab layout: readout strip on top, live plot in the middle, toolbar at bottom."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Numeric readout strip — always visible at the top.
        layout.addWidget(self._build_readout_strip())

        # Scrolling live plot — expands to fill remaining vertical space.
        self._plot = LivePlot(history_s=120.0, telem_hz=10)
        layout.addWidget(self._plot, stretch=1)

        # Toolbar — telem rate, recording controls, export, clear.
        layout.addWidget(self._build_toolbar())

    def _build_readout_strip(self) -> QWidget:
        """Create the horizontal numeric readout strip and return it as a widget.

        Each readout column is a small titled box showing the latest value for
        one telemetry channel.  The columns are added left-to-right in the order:
        position, actual velocity, commanded velocity, acceleration, state, phase,
        frame counter.
        """
        strip: QWidget     = QWidget()
        row:   QHBoxLayout = QHBoxLayout(strip)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)

        def _col(title: str, width: int = 110) -> QLabel:
            """Create one titled readout column, add it to *row*, and return the value label.

            Args:
                title: Header text shown above the numeric value.
                width: Minimum pixel width of the value label.

            Returns:
                The ``QLabel`` whose text will be updated on every telemetry frame.
            """
            title_label: QLabel
            value_label: QLabel
            title_label, value_label = _readout_label(title, width)

            # Stack the title above the value in a tight vertical column.
            column: QVBoxLayout = QVBoxLayout()
            column.setSpacing(1)
            column.addWidget(title_label)
            column.addWidget(value_label)
            row.addLayout(column)
            return value_label

        # One column per telemetry channel, in display order.
        self._lbl_pos      = _col("Position (mm)",          120)
        self._lbl_vel_act  = _col("Vel Actual (mm/s)")
        self._lbl_vel_cmd  = _col("Vel Commanded (mm/s)",   130)
        self._lbl_accel    = _col("Accel (mm/s²)")
        self._lbl_state    = _col("State",                   90)
        self._lbl_phase    = _col("Phase",                  120)
        self._lbl_frames   = _col("Frames",                  80)
        row.addStretch()
        return strip

    def _build_toolbar(self) -> QWidget:
        """Create the bottom toolbar with telem-rate controls, record toggle, PNG export, and clear.

        Returns:
            A ``QWidget`` containing the complete toolbar row.
        """
        bar:    QWidget     = QWidget()
        layout: QHBoxLayout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # --- Telemetry rate section ----------------------------------------
        # Spinbox + "Set" button to send CMD SET_TELEM_RATE to the Arduino.
        layout.addWidget(QLabel("Telem Hz:"))

        self._spin_rate: QSpinBox = QSpinBox()
        self._spin_rate.setRange(1, 50)
        self._spin_rate.setValue(10)
        self._spin_rate.setFixedWidth(60)
        self._spin_rate.setToolTip("Telemetry broadcast rate (1–50 Hz)")
        layout.addWidget(self._spin_rate)

        self._btn_set_rate: QPushButton = QPushButton("Set")
        self._btn_set_rate.setFixedWidth(44)
        self._btn_set_rate.setEnabled(False)   # enabled only when connected
        self._btn_set_rate.setToolTip("Send CMD SET_TELEM_RATE to Arduino")
        self._btn_set_rate.clicked.connect(self._on_set_rate)
        layout.addWidget(self._btn_set_rate)

        layout.addSpacing(12)

        # --- Recording section ---------------------------------------------
        # Auto-record checkbox automatically starts / stops on state transitions.
        self._chk_auto: QCheckBox = QCheckBox("Auto-record")
        self._chk_auto.setChecked(True)
        self._chk_auto.setToolTip(
            "Automatically start/stop CSV recording when state enters/leaves RUNNING"
        )
        layout.addWidget(self._chk_auto)

        # Manual record toggle — turns red while active.
        self._btn_record: QPushButton = QPushButton("Rec ●")
        self._btn_record.setCheckable(True)
        self._btn_record.setFixedWidth(70)
        self._btn_record.setToolTip("Manually start or stop CSV recording")
        self._btn_record.clicked.connect(self._on_record_toggled)
        self._btn_record.setStyleSheet(
            "QPushButton { color: #eeeeee; }"
            "QPushButton:checked { background-color: #c62828; color: white; font-weight: bold; }"
        )
        layout.addWidget(self._btn_record)

        layout.addStretch()

        # --- Right-side utility buttons ------------------------------------
        btn_png: QPushButton = QPushButton("Export PNG")
        btn_png.setToolTip("Save the current plot as a PNG image")
        btn_png.clicked.connect(self._on_export_png)
        layout.addWidget(btn_png)

        btn_clear: QPushButton = QPushButton("Clear")
        btn_clear.setToolTip("Clear the plot (does not affect recording)")
        btn_clear.clicked.connect(self._plot.clear)
        layout.addWidget(btn_clear)

        return bar

    # ------------------------------------------------------------------
    # Private — readout updates
    # ------------------------------------------------------------------

    def _update_readouts(self, frame: TelemetryFrame) -> None:
        """Refresh every numeric readout label from the latest telemetry frame.

        Velocity and acceleration values are sourced from the live plot rather
        than the raw frame so that the displayed values match the smoothed curves
        shown on the plot (EMA-filtered acceleration, ramped commanded velocity).

        Args:
            frame: The most recently received telemetry frame.
        """
        # Raw positional and state fields come directly from the frame.
        self._lbl_pos.setText(f"{frame.pos_mm:.2f}")
        self._lbl_vel_act.setText(f"{frame.vel_actual_mm_s:.2f}")
        self._lbl_state.setText(frame.state)
        self._lbl_phase.setText(frame.phase.replace("_", " "))

        # Velocity (commanded) and acceleration are taken from the plot's
        # derived signals to stay consistent with what is drawn on screen.
        self._lbl_vel_cmd.setText(f"{self._plot.last_vel_cmd_ramped:.2f}")
        self._lbl_accel.setText(f"{self._plot.last_accel_computed:.2f}")

        # Frame counter reflects how many frames the recorder has stored so far.
        self._lbl_frames.setText(str(self._recorder.frame_count))

    # ------------------------------------------------------------------
    # Private — recording lifecycle
    # ------------------------------------------------------------------

    def _handle_recording(self, frame: TelemetryFrame) -> None:
        """Start/stop the recorder automatically based on state transitions."""
        if not self._chk_auto.isChecked():
            return

        entered_running = (frame.state == "RUNNING" and self._last_state != "RUNNING")
        left_running    = (frame.state != "RUNNING" and self._last_state == "RUNNING")

        if entered_running and not self._recorder.is_recording:
            self._start_recording()

        elif left_running and self._recorder.is_recording:
            self._stop_recording()

    def _start_recording(self) -> None:
        """Begin a new CSV recording session tagged with the current profile name.

        Updates the Rec button to its checked (red) state so the operator can
        see that recording is active.
        """
        self._recorder.start(self._profile_name)
        self._btn_record.setChecked(True)
        log.info("Recording started for profile %r", self._profile_name)

    def _stop_recording(self) -> None:
        """Finalise the active recording and flush the CSV to disk.

        Resets the Rec button to its unchecked state.  A ``RuntimeError`` from
        :meth:`~core.data_recorder.DataRecorder.finish` is silently ignored
        because it only fires when no recording is active — a harmless race
        condition between the auto-record logic and a manual stop.
        """
        try:
            run = self._recorder.finish()
            self._btn_record.setChecked(False)
            if run.csv_path:
                log.info("CSV saved: %s (%d frames)", run.csv_path, run.frame_count)
        except RuntimeError:
            pass   # finish() called when no run active — ignore

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_set_rate(self) -> None:
        """Send CMD SET_TELEM_RATE to the Arduino using the spinbox value.

        Shows a warning dialog if the command is rejected (e.g. Arduino not ready).
        """
        if self._ci is None:
            return
        rate_hz: int = self._spin_rate.value()
        try:
            self._ci.set_telem_rate(rate_hz)
        except CommandError as exc:
            QMessageBox.warning(self, "Set rate failed", str(exc))

    def _on_record_toggled(self, checked: bool) -> None:
        """Manually start or stop recording when the operator clicks the Rec button.

        Delegates to :meth:`_start_recording` or :meth:`_stop_recording` as
        appropriate, but only if the recorder is not already in the requested state
        (prevents double-start or double-stop when auto-record also fires).

        Args:
            checked: ``True`` when the button transitions to its pressed (recording) state.
        """
        if checked:
            if not self._recorder.is_recording:
                self._start_recording()
        else:
            if self._recorder.is_recording:
                self._stop_recording()

    def _on_export_png(self) -> None:
        """Save the current plot canvas to a user-chosen PNG file.

        Opens a file-save dialog pre-filled with a default filename.  The plot
        widget is captured using ``QWidget.grab()`` so the exported image matches
        exactly what is shown on screen.  Logs a warning if the write fails.
        """
        # Ask the operator where to save the image.
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save plot as PNG",
            "telemetry_plot.png",
            "PNG Images (*.png)",
        )
        if not path:
            return   # operator cancelled the dialog

        # Capture and write the plot widget to disk.
        pixmap = self._plot.grab()
        if pixmap.save(path, "PNG"):
            log.info("Plot exported: %s", path)
        else:
            QMessageBox.warning(self, "Export failed", f"Could not write {path}")
