"""
ui/widgets/status_bar.py
========================

Horizontal status strip showing live machine state.

Four read-only fields, updated by calling :meth:`StatusBar.update_frame`
whenever a new telemetry frame arrives:

* **Port** — serial port name and connection indicator (green/red dot).
* **State** — Arduino state string (IDLE / HOMING / READY / RUNNING / …).
* **Phase** — run sub-phase (NONE / DESCENDING / DWELL_BOTTOM / …).
* **Elapsed** — wall-clock time spent in RUNNING state, formatted ``MM:SS.t``.

Usage::

    from ui.widgets.status_bar import StatusBar
    from core.telemetry_parser import TelemetryFrame

    bar = StatusBar(port="COM4")
    layout.addWidget(bar)

    # Call from the UI thread whenever a new frame arrives:
    bar.update_frame(frame)

    # Reflect a connection change (no frame needed):
    bar.set_connected(True)
    bar.set_connected(False)

Thread safety
-------------
All public methods must be called from the Qt main thread.
"""

from __future__ import annotations

import time
from typing import Final

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QFrame, QHBoxLayout, QLabel, QWidget

from core.telemetry_parser import TelemetryFrame

# ---------------------------------------------------------------------------
# Colour constants
# ---------------------------------------------------------------------------

_COLOR_CONNECTED:    Final[str] = "#4caf50"   # green
_COLOR_DISCONNECTED: Final[str] = "#f44336"   # red

# State → label background colour.
_STATE_COLORS: Final[dict[str, str]] = {
    "IDLE":    "#555555",
    "HOMING":  "#1565c0",
    "READY":   "#2e7d32",
    "RUNNING": "#00695c",
    "PAUSED":  "#e65100",
    "ERROR":   "#b71c1c",
}
_STATE_COLOR_DEFAULT: Final[str] = "#444444"

_LABEL_STYLE_BASE: Final[str] = (
    "QLabel {{ "
    "  color: #eeeeee; "
    "  font-size: 10pt; "
    "  padding: 2px 8px; "
    "  border-radius: 4px; "
    "  background-color: {bg}; "
    "}}"
)


def _colored_label(text: str, bg: str) -> QLabel:
    """Create a centred ``QLabel`` with a coloured background pill.

    Args:
        text: Initial label text.
        bg:   CSS colour string for the background (e.g. ``"#2e7d32"``).

    Returns:
        A styled, centred ``QLabel`` ready to add to a layout.
    """
    lbl: QLabel = QLabel(text)
    lbl.setStyleSheet(_LABEL_STYLE_BASE.format(bg=bg))
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lbl


class StatusBar(QWidget):
    """
    Horizontal status strip with port, state, phase, and elapsed time.

    Args:
        port:   Serial port name displayed in the port indicator
                (e.g. ``"COM4"``).  Updated via :meth:`set_connected`.
        parent: Optional parent widget.
    """

    def __init__(self, port: str = "COM4", parent=None) -> None:
        super().__init__(parent)
        self._port: str = port

        # Elapsed-time tracking.
        self._running:    bool  = False   # True while state == "RUNNING"
        self._run_start:  float = 0.0     # monotonic time when RUNNING began
        self._elapsed_s:  float = 0.0     # accumulated seconds (survives PAUSE)

        self._build_ui()
        self._timer: QTimer = QTimer(self)
        self._timer.setInterval(100)      # refresh elapsed display 10×/s
        self._timer.timeout.connect(self._tick_elapsed)
        self._timer.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_frame(self, frame: TelemetryFrame) -> None:
        """Refresh all fields from a new telemetry frame.

        Must be called from the Qt main thread.

        Args:
            frame: A fully parsed :class:`~core.telemetry_parser.TelemetryFrame`.
        """
        self._update_state(frame.state)
        self._update_phase(frame.phase)
        self._lbl_pos.setText(f"{frame.pos_mm:+.2f} mm")
        self._lbl_vel.setText(f"{frame.vel_actual_mm_s:.2f} mm/s")
        self._lbl_accel.setText(f"{frame.accel_mm_s2:.1f} mm/s²")

    def set_connected(self, connected: bool, port: str | None = None) -> None:
        """Update the port / connection indicator.

        Args:
            connected: ``True`` = port open; ``False`` = disconnected.
            port:      Override the port name shown in the label.  If
                       ``None``, the name passed to ``__init__`` is kept.
        """
        if port is not None:
            self._port = port

        dot:   str = "●"
        color: str = _COLOR_CONNECTED if connected else _COLOR_DISCONNECTED
        label: str = f"{dot} {self._port}"
        self._lbl_port.setText(label)
        self._lbl_port.setStyleSheet(
            _LABEL_STYLE_BASE.format(bg=_STATE_COLOR_DEFAULT)
            + f" QLabel {{ color: {color}; }}"
        )

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        """Lay out the four indicator labels separated by thin vertical lines."""
        layout: QHBoxLayout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(6)

        # Port indicator — shows the port name with a coloured connection dot.
        self._lbl_port: QLabel = _colored_label(f"● {self._port}", _STATE_COLOR_DEFAULT)
        self._lbl_port.setToolTip("Serial port connection status")

        # State label — background colour changes with each Arduino state.
        self._lbl_state: QLabel = _colored_label(
            "IDLE", _STATE_COLORS.get("IDLE", _STATE_COLOR_DEFAULT)
        )
        self._lbl_state.setMinimumWidth(90)
        self._lbl_state.setToolTip("Arduino machine state")

        # Phase label — shows the current run sub-phase (e.g. DESCENDING).
        self._lbl_phase: QLabel = _colored_label("NONE", _STATE_COLOR_DEFAULT)
        self._lbl_phase.setMinimumWidth(110)
        self._lbl_phase.setToolTip("Current run phase")

        # Elapsed time — counts wall-clock seconds spent in RUNNING state.
        self._lbl_elapsed: QLabel = _colored_label("00:00.0", _STATE_COLOR_DEFAULT)
        self._lbl_elapsed.setMinimumWidth(80)
        self._lbl_elapsed.setToolTip("Time spent in RUNNING state")

        # Live telemetry readouts — updated on every frame.
        self._lbl_pos: QLabel = _colored_label("+0.00 mm", _STATE_COLOR_DEFAULT)
        self._lbl_pos.setMinimumWidth(90)
        self._lbl_pos.setToolTip("Current encoder position")

        self._lbl_vel: QLabel = _colored_label("0.00 mm/s", _STATE_COLOR_DEFAULT)
        self._lbl_vel.setMinimumWidth(90)
        self._lbl_vel.setToolTip("Current shaft velocity (encoder)")

        self._lbl_accel: QLabel = _colored_label("0.0 mm/s²", _STATE_COLOR_DEFAULT)
        self._lbl_accel.setMinimumWidth(95)
        self._lbl_accel.setToolTip("Current acceleration")

        def _sep() -> QFrame:
            """Return a thin vertical QFrame line used as a visual separator."""
            separator: QFrame = QFrame()
            separator.setFrameShape(QFrame.Shape.VLine)
            separator.setStyleSheet("QFrame { color: #555555; }")
            return separator

        layout.addWidget(self._lbl_port)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_state)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_phase)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_elapsed)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_pos)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_vel)
        layout.addWidget(_sep())
        layout.addWidget(self._lbl_accel)
        layout.addStretch()

    # ------------------------------------------------------------------
    # Private — field updates
    # ------------------------------------------------------------------

    def _update_state(self, state: str) -> None:
        """Update the state label text and colour; advance the elapsed-time state machine.

        The elapsed-time counter follows these rules:

        * **RUNNING** — start counting (or resume if previously paused).
        * **PAUSED** — freeze the counter without resetting it so the total
          accumulated time is preserved when the run resumes.
        * **Any other state** (IDLE, READY, HOMING, ERROR) — reset the counter
          to zero because a new run has not yet started or has ended.

        Args:
            state: The latest Arduino state string (e.g. ``"RUNNING"``).
        """
        # Update the label text and its background colour.
        self._lbl_state.setText(state)
        background_color: str = _STATE_COLORS.get(state, _STATE_COLOR_DEFAULT)
        self._lbl_state.setStyleSheet(_LABEL_STYLE_BASE.format(bg=background_color))

        # Advance the elapsed-time state machine.
        if state == "RUNNING":
            if not self._running:
                # Transition into RUNNING — start the stopwatch.
                self._running   = True
                self._run_start = time.monotonic()
        elif state == "PAUSED":
            if self._running:
                # Transition into PAUSED — bank elapsed time and freeze the clock.
                self._elapsed_s += time.monotonic() - self._run_start
                self._running    = False
        else:
            # Any non-RUNNING / non-PAUSED state — reset the elapsed counter.
            self._running   = False
            self._elapsed_s = 0.0
            self._lbl_elapsed.setText("00:00.0")

    def _update_phase(self, phase: str) -> None:
        """Update the phase label, replacing underscores with spaces for readability.

        Args:
            phase: The latest Arduino phase string (e.g. ``"DWELL_BOTTOM"``).
        """
        self._lbl_phase.setText(phase.replace("_", " "))

    def _tick_elapsed(self) -> None:
        """Called by QTimer 10×/s to refresh the elapsed display while running."""
        if not self._running:
            return
        total_s: float = self._elapsed_s + (time.monotonic() - self._run_start)
        minutes: int   = int(total_s) // 60
        seconds: float = total_s - minutes * 60
        self._lbl_elapsed.setText(f"{minutes:02d}:{seconds:04.1f}")
