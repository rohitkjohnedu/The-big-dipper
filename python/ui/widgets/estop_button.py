"""
ui/widgets/estop_button.py
==========================

Large, always-visible Emergency Stop button.

Clicking it calls :meth:`~core.command_interface.CommandInterface.estop`
directly with no confirmation dialog and no delay.  The button cannot be
disabled programmatically — disabling an E-stop defeats the point.

Usage::

    from core.command_interface import CommandInterface
    from ui.widgets.estop_button import EstopButton

    button = EstopButton(command_interface=ci)
    layout.addWidget(button)

The standard ``clicked`` signal (inherited from ``QPushButton``) fires
*after* ``ci.estop()`` so callers can connect additional slots (e.g. to
grey-out the Run button) without sub-classing.
"""

from __future__ import annotations

import logging
from typing import Final

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QPushButton, QSizePolicy

from core.command_interface import CommandInterface

log: Final[logging.Logger] = logging.getLogger(__name__)

_MIN_WIDTH_PX:  Final[int] = 160
_MIN_HEIGHT_PX: Final[int] = 64

_STYLE: Final[str] = """
QPushButton {
    background-color: #CC0000;
    color: #FFFFFF;
    font-size: 16pt;
    font-weight: bold;
    border: 4px solid #880000;
    border-radius: 10px;
    padding: 10px 20px;
}
QPushButton:pressed {
    background-color: #880000;
    border-color: #440000;
}
"""


class EstopButton(QPushButton):
    """
    Large red Emergency Stop button wired to :meth:`CommandInterface.estop`.

    Args:
        command_interface: A live :class:`~core.command_interface.CommandInterface`
                           instance whose :meth:`~core.command_interface.CommandInterface.estop`
                           is called on every click.  May be ``None`` when
                           disconnected — clicks are silently ignored.
        parent:            Optional parent widget.
    """

    def __init__(
        self,
        command_interface: "CommandInterface | None",
        parent=None,
    ) -> None:
        super().__init__("EMERGENCY STOP", parent)
        self._ci: "CommandInterface | None" = command_interface
        self._apply_style()
        self.clicked.connect(self._on_clicked)

    # ------------------------------------------------------------------

    def set_command_interface(self, ci: "CommandInterface | None") -> None:
        """Update the target command interface (call on connect / disconnect)."""
        self._ci = ci

    def _apply_style(self) -> None:
        self.setStyleSheet(_STYLE)
        self.setMinimumWidth(_MIN_WIDTH_PX)
        self.setMinimumHeight(_MIN_HEIGHT_PX)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip(
            "Emergency Stop — immediately cuts motor power.\n"
            "Home the carriage before resuming operation."
        )

    def _on_clicked(self) -> None:
        if self._ci is None:
            log.warning("EstopButton clicked but no command interface set — ignored")
            return
        log.warning("EstopButton clicked — sending ESTOP")
        self._ci.estop()
