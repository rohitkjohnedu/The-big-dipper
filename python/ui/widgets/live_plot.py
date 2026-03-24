"""
ui/widgets/live_plot.py
=======================

Real-time position / velocity / acceleration plot fed by telemetry frames.

Three vertically stacked pyqtgraph subplots share a linked x-axis (time in
seconds).  A control toolbar sits above the plots with:

* **Home** — re-enables auto-range on all subplots after the user has
  manually panned or zoomed.
* **Window / Full History** — toggle between a rolling window (latest N
  seconds) and full-history mode (every frame since the last ``clear()``).
* **Window spinbox** — adjustable window duration (seconds), active in
  Window mode only.

In **Window mode** data is held in fixed-length :class:`collections.deque`
buffers (``_t``, ``_pos``, etc.) so memory is bounded regardless of run
duration.  In **Full History mode** all frames are held in companion
unbounded lists (``_all_t``, ``_all_pos``, etc.).

Usage::

    from ui.widgets.live_plot import LivePlot
    from core.telemetry_parser import TelemetryFrame

    plot = LivePlot(history_s=60.0, telem_hz=10)
    layout.addWidget(plot)

    # Call from the UI thread whenever a new frame arrives:
    plot.push_frame(frame)

    # Reset between runs:
    plot.clear()

Thread safety
-------------
:meth:`push_frame` and :meth:`clear` must be called from the Qt main thread.
Use a ``pyqtSignal`` to forward frames from the serial-reader thread to the
main thread before calling these methods.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Final

import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QHBoxLayout, QPushButton, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from core.telemetry_parser import TelemetryFrame

# ---------------------------------------------------------------------------
# Visual constants
# ---------------------------------------------------------------------------

_BG_COLOR:        Final[str] = "#1a1a1a"
_GRID_ALPHA:      Final[int] = 40          # 0–255
_PEN_WIDTH:       Final[int] = 2

_COLOR_POS:       Final[str] = "#4fc3f7"   # light blue  — position
_COLOR_VEL_CMD:   Final[str] = "#ffb74d"   # amber       — commanded velocity
_COLOR_VEL_ACT:   Final[str] = "#aed581"   # light green — actual velocity (dashed)
_COLOR_ACCEL:     Final[str] = "#f06292"   # pink        — acceleration

_LABEL_STYLE:     Final[dict] = {"color": "#cccccc", "font-size": "9pt"}


class LivePlot(QWidget):
    """
    Three-panel live telemetry plot with Home and history-mode controls.

    Panels (top to bottom):

    1. **Position** (mm) — single curve, light blue.
    2. **Velocity** (mm/s) — commanded (amber) and actual (green dashed).
    3. **Acceleration** (mm/s²) — single curve, pink.

    Args:
        history_s:  Rolling window width in seconds used in Window mode and
                    as the initial spinbox value.  Default: 60 s.
        telem_hz:   Expected telemetry rate used to size the rolling deques.
                    Actual data rate does not need to match exactly.
                    Default: 10 Hz.
        parent:     Optional parent widget.
    """

    def __init__(
        self,
        history_s: float = 60.0,
        telem_hz:  int   = 10,
        parent=None,
    ) -> None:
        super().__init__(parent)

        self._history_s: float = history_s
        maxlen: int = int(history_s * telem_hz) + 1

        # --- Rolling buffers (Window mode) ----------------------------------
        # Bounded by maxlen — old frames are dropped when the window is full.
        self._t:       deque[float] = deque(maxlen=maxlen)
        self._pos:     deque[float] = deque(maxlen=maxlen)
        self._vel_cmd: deque[float] = deque(maxlen=maxlen)
        self._vel_act: deque[float] = deque(maxlen=maxlen)
        self._accel:   deque[float] = deque(maxlen=maxlen)

        # --- Full-history buffers (Full History mode) ----------------------
        # Unbounded — every frame since the last clear() is kept.
        self._all_t:       list[float] = []
        self._all_pos:     list[float] = []
        self._all_vel_cmd: list[float] = []
        self._all_vel_act: list[float] = []
        self._all_accel:   list[float] = []

        self._t0: float | None = None   # timestamp_ms of the first frame (x=0)

        # --- Derived-signal state -----------------------------------------
        self._prev_t_ms:      float | None = None
        self._prev_vel_actual: float | None = None
        self._accel_ema:      float        = 0.0   # EMA-smoothed dv/dt
        self._vel_cmd_display: float       = 0.0   # ramped commanded vel

        self._build_ui()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def push_frame(self, frame: TelemetryFrame) -> None:
        """Append one telemetry frame and redraw all curves.

        Must be called from the Qt main thread.

        Args:
            frame: A fully parsed :class:`~core.telemetry_parser.TelemetryFrame`.
        """
        if self._t0 is None:
            self._t0 = frame.timestamp_ms

        t_s: float = (frame.timestamp_ms - self._t0) / 1000.0

        # --- dt from previous frame ----------------------------------------
        dt = (frame.timestamp_ms - self._prev_t_ms) / 1000.0 \
             if self._prev_t_ms is not None else 0.0

        vel_actual = frame.vel_actual_mm_s

        # --- Noise floor: show commanded=0 when motor is stationary ---------
        # Firmware keeps last segment speed in vel_commanded during dwells.
        _VEL_NOISE = 0.5  # mm/s
        if abs(vel_actual) < _VEL_NOISE:
            vel_cmd_target = 0.0
        else:
            vel_cmd_target = math.copysign(frame.vel_commanded_mm_s, vel_actual)

        # --- EMA-smoothed acceleration from encoder velocity derivative ------
        if dt > 1e-6 and self._prev_vel_actual is not None:
            raw_accel = (vel_actual - self._prev_vel_actual) / dt
            self._accel_ema = 0.25 * raw_accel + 0.75 * self._accel_ema

        # --- Ramp vel_cmd_display toward target at accel rate ----------------
        # On the very first frame dt=0 — leave _vel_cmd_display at 0 so the
        # ramp engages from the next real frame interval.
        _MIN_RAMP_RATE = 20.0  # mm/s² minimum ramp rate when accel_ema ≈ 0
        if dt > 1e-6:
            ramp_rate = max(abs(self._accel_ema), _MIN_RAMP_RATE)
            delta     = vel_cmd_target - self._vel_cmd_display
            max_step  = ramp_rate * dt
            if abs(delta) <= max_step:
                self._vel_cmd_display = vel_cmd_target
            else:
                self._vel_cmd_display += math.copysign(max_step, delta)

        vel_cmd   = self._vel_cmd_display
        accel_out = self._accel_ema

        self._prev_t_ms       = frame.timestamp_ms
        self._prev_vel_actual = vel_actual

        # Rolling buffers
        self._t.append(t_s)
        self._pos.append(frame.pos_mm)
        self._vel_cmd.append(vel_cmd)
        self._vel_act.append(vel_actual)
        self._accel.append(accel_out)

        # Full-history buffers
        self._all_t.append(t_s)
        self._all_pos.append(frame.pos_mm)
        self._all_vel_cmd.append(vel_cmd)
        self._all_vel_act.append(vel_actual)
        self._all_accel.append(accel_out)

        self._redraw()

    def clear(self) -> None:
        """Reset all buffers and blank the plot (call between runs)."""
        self._t0 = None
        self._prev_t_ms       = None
        self._prev_vel_actual = None
        self._accel_ema       = 0.0
        self._vel_cmd_display = 0.0
        self._t.clear()
        self._pos.clear()
        self._vel_cmd.clear()
        self._vel_act.clear()
        self._accel.clear()
        self._all_t.clear()
        self._all_pos.clear()
        self._all_vel_cmd.clear()
        self._all_vel_act.clear()
        self._all_accel.clear()
        self._redraw()

    @property
    def last_accel_computed(self) -> float:
        """Most recent EMA-smoothed acceleration (mm/s²), derived from encoder velocity."""
        return self._accel_ema

    @property
    def last_vel_cmd_ramped(self) -> float:
        """Most recent ramped commanded velocity (mm/s) for display in readouts."""
        return self._vel_cmd_display

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        pg.setConfigOption("background", _BG_COLOR)
        pg.setConfigOption("foreground", "#cccccc")

        self._glw = pg.GraphicsLayoutWidget()
        self._glw.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )

        # --- Position subplot -------------------------------------------
        self._p_pos = self._glw.addPlot(row=0, col=0)
        self._p_pos.setLabel("left",   "Position",     units="mm",   **_LABEL_STYLE)
        self._p_pos.setLabel("bottom", "Time",         units="s",    **_LABEL_STYLE)
        self._p_pos.showGrid(x=True, y=True, alpha=_GRID_ALPHA / 255)
        self._p_pos.addLegend(offset=(5, 5))
        self._curve_pos = self._p_pos.plot(
            pen=pg.mkPen(_COLOR_POS, width=_PEN_WIDTH),
            name="position",
        )

        # --- Velocity subplot --------------------------------------------
        self._p_vel = self._glw.addPlot(row=1, col=0)
        self._p_vel.setLabel("left",   "Velocity",     units="mm/s", **_LABEL_STYLE)
        self._p_vel.setLabel("bottom", "Time",         units="s",    **_LABEL_STYLE)
        self._p_vel.showGrid(x=True, y=True, alpha=_GRID_ALPHA / 255)
        self._p_vel.addLegend(offset=(5, 5))
        self._curve_vel_cmd = self._p_vel.plot(
            pen=pg.mkPen(_COLOR_VEL_CMD, width=_PEN_WIDTH),
            name="commanded",
        )
        self._curve_vel_act = self._p_vel.plot(
            pen=pg.mkPen(_COLOR_VEL_ACT, width=_PEN_WIDTH,
                         style=pg.QtCore.Qt.PenStyle.DashLine),
            name="actual",
        )

        # --- Acceleration subplot ----------------------------------------
        self._p_accel = self._glw.addPlot(row=2, col=0)
        self._p_accel.setLabel("left",   "Acceleration", units="mm/s²", **_LABEL_STYLE)
        self._p_accel.setLabel("bottom", "Time",         units="s",     **_LABEL_STYLE)
        self._p_accel.showGrid(x=True, y=True, alpha=_GRID_ALPHA / 255)
        self._curve_accel = self._p_accel.plot(
            pen=pg.mkPen(_COLOR_ACCEL, width=_PEN_WIDTH),
            name="accel",
        )

        # Link all x-axes to the position plot so zooming one zooms all.
        self._p_vel.setXLink(self._p_pos)
        self._p_accel.setXLink(self._p_pos)

        # --- Control toolbar --------------------------------------------
        toolbar = self._build_toolbar()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(toolbar)
        layout.addWidget(self._glw, stretch=1)

    def _build_toolbar(self) -> QWidget:
        bar    = QWidget()
        row    = QHBoxLayout(bar)
        row.setContentsMargins(4, 2, 4, 2)
        row.setSpacing(6)

        # Home button
        btn_home = QPushButton("Home")
        btn_home.setToolTip(
            "Reset view — re-enable auto-range on all plots after panning/zooming"
        )
        btn_home.setFixedWidth(60)
        btn_home.clicked.connect(self._on_home)
        row.addWidget(btn_home)

        row.addSpacing(12)

        # Window mode button (default on)
        self._btn_window = QPushButton("Window")
        self._btn_window.setCheckable(True)
        self._btn_window.setChecked(True)
        self._btn_window.setToolTip("Show the latest N seconds of data")
        self._btn_window.setFixedWidth(72)
        self._btn_window.clicked.connect(self._on_window_mode)
        row.addWidget(self._btn_window)

        # Window duration spinbox
        self._spin_window = QSpinBox()
        self._spin_window.setRange(5, 3600)
        self._spin_window.setValue(int(self._history_s))
        self._spin_window.setSuffix(" s")
        self._spin_window.setFixedWidth(76)
        self._spin_window.setToolTip("Rolling window duration (Window mode only)")
        self._spin_window.valueChanged.connect(self._on_window_duration_changed)
        row.addWidget(self._spin_window)

        # Full History button
        self._btn_full = QPushButton("Full History")
        self._btn_full.setCheckable(True)
        self._btn_full.setChecked(False)
        self._btn_full.setToolTip("Show all data since the last Clear")
        self._btn_full.setFixedWidth(96)
        self._btn_full.clicked.connect(self._on_full_history_mode)
        row.addWidget(self._btn_full)

        row.addStretch()

        self._update_mode_style()
        return bar

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_home(self) -> None:
        """Re-enable auto-range on all subplots (resets after manual pan/zoom)."""
        for p in (self._p_pos, self._p_vel, self._p_accel):
            p.enableAutoRange()

    def _on_window_mode(self) -> None:
        """Switch to Window (rolling) mode."""
        self._btn_window.setChecked(True)
        self._btn_full.setChecked(False)
        self._spin_window.setEnabled(True)
        self._update_mode_style()
        self._redraw()

    def _on_full_history_mode(self) -> None:
        """Switch to Full History mode."""
        self._btn_full.setChecked(True)
        self._btn_window.setChecked(False)
        self._spin_window.setEnabled(False)
        self._update_mode_style()
        self._redraw()

    def _on_window_duration_changed(self, value: int) -> None:
        """Redraw immediately when the window spinbox value changes."""
        if self._btn_window.isChecked():
            self._redraw()

    def _update_mode_style(self) -> None:
        active   = "QPushButton { background-color: #1565c0; color: white; font-weight: bold; }"
        inactive = ""
        self._btn_window.setStyleSheet(active if self._btn_window.isChecked() else inactive)
        self._btn_full.setStyleSheet(active   if self._btn_full.isChecked()   else inactive)

    # ------------------------------------------------------------------
    # Private — redraw
    # ------------------------------------------------------------------

    def _redraw(self) -> None:
        if self._btn_full.isChecked():
            # Full history — use all data collected since last clear()
            t        = self._all_t
            pos      = self._all_pos
            vel_cmd  = self._all_vel_cmd
            vel_act  = self._all_vel_act
            accel    = self._all_accel
        else:
            # Window mode — use the rolling deques (already bounded by maxlen)
            t        = list(self._t)
            pos      = list(self._pos)
            vel_cmd  = list(self._vel_cmd)
            vel_act  = list(self._vel_act)
            accel    = list(self._accel)

            # Trim to the spinbox window if data is older than requested duration
            win_s = float(self._spin_window.value())
            if t and (t[-1] - t[0]) > win_s:
                import bisect
                cutoff = t[-1] - win_s
                idx    = bisect.bisect_left(t, cutoff)
                t       = t[idx:]
                pos     = pos[idx:]
                vel_cmd = vel_cmd[idx:]
                vel_act = vel_act[idx:]
                accel   = accel[idx:]

        self._curve_pos.setData(t, pos)
        self._curve_vel_cmd.setData(t, vel_cmd)
        self._curve_vel_act.setData(t, vel_act)
        self._curve_accel.setData(t, accel)
