"""
ui/tabs/profile_editor_tab.py
=============================

Stub for the spline velocity-curve editor (Step 27).

This tab will eventually provide a graphical waypoint editor that lets the
operator draw a custom velocity-vs-position curve, which is then converted
to a ``SplineProfile`` and dispatched as a segmented move.

The implementation is deferred until the full UI is assembled (Step 27).
The stub exists so that:

* ``from ui.tabs.profile_editor_tab import ProfileEditorTab`` never raises
  ``ImportError``.
* ``MainWindow`` can include the tab in its ``QTabWidget`` without
  conditional imports.
* The tab slot is visible in the UI with a clear "not implemented" message
  rather than being silently absent.
"""

from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget


class ProfileEditorTab(QWidget):
    """
    Placeholder for the spline profile editor.

    Displays an informational message.  No interactive controls are provided
    until Step 27 implements the full spline waypoint editor.

    Args:
        parent: Optional parent widget.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        lbl = QLabel(
            "Spline profile editor\n\nNot yet implemented — coming in Step 27."
        )
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet("color: #888888; font-size: 11pt;")

        layout = QVBoxLayout(self)
        layout.addStretch()
        layout.addWidget(lbl)
        layout.addStretch()
