"""
ui/widgets/live_plot.py
=======================

Real-time position / velocity / acceleration plot fed by telemetry frames.

Three vertically stacked pyqtgraph subplots share a linked x-axis (time in
seconds).  Data is held in fixed-length :class:`collections.deque` buffers so
memory use is bounded regardless of run duration.

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

from collections import deque
from typing import Final

import pyqtgraph as pg
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QSizePolicy, QVBoxLayout, QWidget

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
    Three-panel live telemetry plot.

    Panels (top to bottom):

    1. **Position** (mm) — single curve, light blue.
    2. **Velocity** (mm/s) — commanded (amber) and actual (green dashed).
    3. **Acceleration** (mm/s²) — single curve, pink.

    Args:
        history_s:  Rolling window width in seconds.  Frames older than this
                    are discarded.  Default: 60 s.
        telem_hz:   Expected telemetry rate used to size the deque.  Actual
                    data rate does not need to match exactly.  Default: 10 Hz.
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

        # Rolling data buffers — one entry per telemetry frame.
        self._t:       deque[float] = deque(maxlen=maxlen)
        self._pos:     deque[float] = deque(maxlen=maxlen)
        self._vel_cmd: deque[float] = deque(maxlen=maxlen)
        self._vel_act: deque[float] = deque(maxlen=maxlen)
        self._accel:   deque[float] = deque(maxlen=maxlen)

        self._t0: float | None = None   # timestamp_ms of the first frame (for x=0)

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

        self._t.append(t_s)
        self._pos.append(frame.pos_mm)
        self._vel_cmd.append(frame.vel_commanded_mm_s)
        self._vel_act.append(frame.vel_actual_mm_s)
        self._accel.append(frame.accel_mm_s2)

        self._redraw()

    def clear(self) -> None:
        """Reset all buffers and blank the plot (call between runs)."""
        self._t0 = None
        self._t.clear()
        self._pos.clear()
        self._vel_cmd.clear()
        self._vel_act.clear()
        self._accel.clear()
        self._redraw()

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

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._glw)

    # ------------------------------------------------------------------
    # Private — redraw
    # ------------------------------------------------------------------

    def _redraw(self) -> None:
        t = list(self._t)
        self._curve_pos.setData(t, list(self._pos))
        self._curve_vel_cmd.setData(t, list(self._vel_cmd))
        self._curve_vel_act.setData(t, list(self._vel_act))
        self._curve_accel.setData(t, list(self._accel))
