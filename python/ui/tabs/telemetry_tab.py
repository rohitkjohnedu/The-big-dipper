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
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        layout.addWidget(self._build_readout_strip())
        self._plot = LivePlot(history_s=120.0, telem_hz=10)
        layout.addWidget(self._plot, stretch=1)
        layout.addWidget(self._build_toolbar())

    def _build_readout_strip(self) -> QWidget:
        strip = QWidget()
        row   = QHBoxLayout(strip)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)

        def _col(title: str, width: int = 110) -> QLabel:
            t: QLabel
            v: QLabel
            t, v             = _readout_label(title, width)
            col: QVBoxLayout = QVBoxLayout()
            col.setSpacing(1)
            col.addWidget(t)
            col.addWidget(v)
            row.addLayout(col)
            return v

        self._lbl_pos      = _col("Position (mm)",  120)
        self._lbl_vel_act  = _col("Vel Actual (mm/s)")
        self._lbl_vel_cmd  = _col("Vel Commanded (mm/s)", 130)
        self._lbl_accel    = _col("Accel (mm/s²)")
        self._lbl_state    = _col("State",  90)
        self._lbl_phase    = _col("Phase", 120)
        self._lbl_frames   = _col("Frames",  80)
        row.addStretch()
        return strip

    def _build_toolbar(self) -> QWidget:
        bar    = QWidget()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Telem rate
        layout.addWidget(QLabel("Telem Hz:"))
        self._spin_rate = QSpinBox()
        self._spin_rate.setRange(1, 50)
        self._spin_rate.setValue(10)
        self._spin_rate.setFixedWidth(60)
        self._spin_rate.setToolTip("Telemetry broadcast rate (1–50 Hz)")
        layout.addWidget(self._spin_rate)

        self._btn_set_rate = QPushButton("Set")
        self._btn_set_rate.setFixedWidth(44)
        self._btn_set_rate.setEnabled(False)
        self._btn_set_rate.setToolTip("Send CMD SET_TELEM_RATE to Arduino")
        self._btn_set_rate.clicked.connect(self._on_set_rate)
        layout.addWidget(self._btn_set_rate)

        layout.addSpacing(12)

        # Auto-record
        self._chk_auto = QCheckBox("Auto-record")
        self._chk_auto.setChecked(True)
        self._chk_auto.setToolTip(
            "Automatically start/stop CSV recording when state enters/leaves RUNNING"
        )
        layout.addWidget(self._chk_auto)

        # Manual record toggle
        self._btn_record = QPushButton("Rec ●")
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

        # Export PNG
        btn_png = QPushButton("Export PNG")
        btn_png.setToolTip("Save the current plot as a PNG image")
        btn_png.clicked.connect(self._on_export_png)
        layout.addWidget(btn_png)

        # Clear
        btn_clear = QPushButton("Clear")
        btn_clear.setToolTip("Clear the plot (does not affect recording)")
        btn_clear.clicked.connect(self._plot.clear)
        layout.addWidget(btn_clear)

        return bar

    # ------------------------------------------------------------------
    # Private — readout updates
    # ------------------------------------------------------------------

    def _update_readouts(self, frame: TelemetryFrame) -> None:
        self._lbl_pos.setText(f"{frame.pos_mm:.2f}")
        self._lbl_vel_act.setText(f"{frame.vel_actual_mm_s:.2f}")
        self._lbl_vel_cmd.setText(f"{self._plot.last_vel_cmd_ramped:.2f}")
        self._lbl_accel.setText(f"{self._plot.last_accel_computed:.2f}")
        self._lbl_state.setText(frame.state)
        self._lbl_phase.setText(frame.phase.replace("_", " "))
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
        self._recorder.start(self._profile_name)
        self._btn_record.setChecked(True)
        log.info("Recording started for profile %r", self._profile_name)

    def _stop_recording(self) -> None:
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
        if self._ci is None:
            return
        hz = self._spin_rate.value()
        try:
            self._ci.set_telem_rate(hz)
        except CommandError as exc:
            QMessageBox.warning(self, "Set rate failed", str(exc))

    def _on_record_toggled(self, checked: bool) -> None:
        if checked:
            if not self._recorder.is_recording:
                self._start_recording()
        else:
            if self._recorder.is_recording:
                self._stop_recording()

    def _on_export_png(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save plot as PNG",
            "telemetry_plot.png",
            "PNG Images (*.png)",
        )
        if not path:
            return
        pixmap = self._plot.grab()
        if pixmap.save(path, "PNG"):
            log.info("Plot exported: %s", path)
        else:
            QMessageBox.warning(self, "Export failed", f"Could not write {path}")
