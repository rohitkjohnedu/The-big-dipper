"""
ui/tabs/profile_manager_tab.py
==============================

Profile manager tab — browse, create, duplicate, rename, and delete JSON
dip-coating profiles stored in the ``profiles/`` directory.

Layout
------
::

    ┌────────────────────────────────────────────────────────────────────┐
    │  [Refresh]                                                         │  ← top toolbar
    ├──────────┬─────────────────────────────────────────────────────────┤
    │ Name     │ Dip (mm/s) │ Wdraw (mm/s) │ Depth (mm) │ Dips │ Created │  ← header
    │ ...      │ ...        │ ...          │ ...        │ ...  │ ...     │
    │ ...      │ ...        │ ...          │ ...        │ ...  │ ...     │
    ├────────────────────────────────────────────────────────────────────┤
    │ [New]  [Duplicate]  [Rename]  [Delete]          [Load]            │  ← bottom toolbar
    └────────────────────────────────────────────────────────────────────┘

Public API (called by ``MainWindow``)::

    tab.set_profile_dir(path)            # set/change the profiles directory
    tab.refresh()                        # reload profiles from disk
    # Signal
    tab.profile_selected(DipProfile)     # emitted when Load is clicked

Profile directory
-----------------
The tab scans ``profiles/`` (relative to the working directory) for ``*.json``
files.  A missing directory is created automatically on first use.  A corrupt
file is skipped with a warning in the table (row shown in red).

New profile dialog
------------------
Clicking **New** opens :class:`_NewProfileDialog` — a form with all
``DipProfile`` required fields.  On accept the profile is saved to disk and
the table is refreshed.  Validation errors from ``DipProfile.__post_init__``
are shown in a ``QMessageBox``.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Final, Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QAbstractItemView, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMessageBox, QPushButton, QSpinBox, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from core.profile import DipProfile, list_profiles, load_profile, save_profile

log: Final[logging.Logger] = logging.getLogger(__name__)

# Column indices
_COL_NAME    = 0
_COL_DIP     = 1
_COL_WDRAW   = 2
_COL_ACCEL   = 3
_COL_DEPTH   = 4
_COL_DIPS    = 5
_COL_CREATED = 6
_NUM_COLS    = 7

_HEADERS = ("Name", "Dip (mm/s)", "Wdraw (mm/s)", "Accel (mm/s²)",
            "Depth (mm)", "Dips", "Created")


# ---------------------------------------------------------------------------
# New-profile dialog
# ---------------------------------------------------------------------------

class _NewProfileDialog(QDialog):
    """Modal form for creating a new DipProfile."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("New Profile")
        self.setMinimumWidth(360)

        form = QFormLayout()
        form.setSpacing(8)

        self._name        = QLineEdit()
        self._name.setPlaceholderText("e.g. silica_coat_v1")
        self._dip_spd     = QDoubleSpinBox(); self._dip_spd.setRange(0.1, 100.0); self._dip_spd.setValue(5.0);   self._dip_spd.setSuffix(" mm/s")
        self._wdraw_spd   = QDoubleSpinBox(); self._wdraw_spd.setRange(0.1, 100.0); self._wdraw_spd.setValue(5.0); self._wdraw_spd.setSuffix(" mm/s")
        self._accel       = QDoubleSpinBox(); self._accel.setRange(1.0, 500.0); self._accel.setValue(20.0);      self._accel.setSuffix(" mm/s²")
        self._depth       = QDoubleSpinBox(); self._depth.setRange(0.1, 880.0); self._depth.setValue(20.0);      self._depth.setSuffix(" mm")
        self._dwell_bot   = QSpinBox();       self._dwell_bot.setRange(0, 60_000); self._dwell_bot.setValue(500);  self._dwell_bot.setSuffix(" ms")
        self._dwell_top   = QSpinBox();       self._dwell_top.setRange(0, 60_000); self._dwell_top.setValue(200);  self._dwell_top.setSuffix(" ms")
        self._n_dips      = QSpinBox();       self._n_dips.setRange(1, 100); self._n_dips.setValue(1)
        self._notes       = QLineEdit()
        self._notes.setPlaceholderText("Optional notes")

        form.addRow("Name *",          self._name)
        form.addRow("Dip speed *",     self._dip_spd)
        form.addRow("Withdraw speed *",self._wdraw_spd)
        form.addRow("Acceleration *",  self._accel)
        form.addRow("Dip depth *",     self._depth)
        form.addRow("Dwell bottom",    self._dwell_bot)
        form.addRow("Dwell top",       self._dwell_top)
        form.addRow("# Dips *",        self._n_dips)
        form.addRow("Notes",           self._notes)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    def get_profile(self) -> DipProfile:
        """Return a DipProfile built from the form values.

        Raises:
            ValueError: If any field fails DipProfile validation.
        """
        return DipProfile(
            name                 = self._name.text().strip(),
            dip_speed_mm_s       = self._dip_spd.value(),
            withdraw_speed_mm_s  = self._wdraw_spd.value(),
            accel_mm_s2          = self._accel.value(),
            dip_depth_mm         = self._depth.value(),
            dwell_bottom_ms      = self._dwell_bot.value(),
            dwell_top_ms         = self._dwell_top.value(),
            n_dips               = self._n_dips.value(),
            notes                = self._notes.text(),
        )


# ---------------------------------------------------------------------------
# ProfileManagerTab
# ---------------------------------------------------------------------------

class ProfileManagerTab(QWidget):
    """
    Browse, create, duplicate, rename, and delete JSON profiles.

    Args:
        profile_dir: Path to the ``profiles/`` directory.  Created if absent.
        parent:      Optional parent widget.

    Signals:
        profile_selected(DipProfile): Emitted when the operator clicks **Load**.
    """

    profile_selected: pyqtSignal = pyqtSignal(object)

    def __init__(
        self,
        profile_dir: str | Path = "profiles",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._profile_dir = Path(profile_dir)
        self._profiles:  list[DipProfile] = []

        self._build_ui()
        self.refresh()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_profile_dir(self, path: str | Path) -> None:
        """Change the profile directory and refresh the table."""
        self._profile_dir = Path(path)
        self.refresh()

    def refresh(self) -> None:
        """Reload all profiles from disk and repopulate the table."""
        self._profile_dir.mkdir(parents=True, exist_ok=True)
        self._profiles = list_profiles(self._profile_dir)
        self._populate_table()

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        layout.addWidget(self._build_top_toolbar())
        layout.addWidget(self._build_table(), stretch=1)
        layout.addWidget(self._build_bottom_toolbar())

    def _build_top_toolbar(self) -> QWidget:
        bar    = QWidget()
        row    = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)

        self._btn_refresh = QPushButton("Refresh")
        self._btn_refresh.setToolTip("Reload profiles from disk")
        self._btn_refresh.clicked.connect(self.refresh)
        row.addWidget(self._btn_refresh)
        row.addStretch()

        self._lbl_dir = QLabel()
        self._lbl_dir.setStyleSheet("color: #888888; font-size: 8pt;")
        row.addWidget(self._lbl_dir)
        self._lbl_dir.setText(str(self._profile_dir))

        return bar

    def _build_table(self) -> QTableWidget:
        self._table = QTableWidget(0, _NUM_COLS)
        self._table.setHorizontalHeaderLabels(_HEADERS)
        self._table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self._table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self._table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self._table.setAlternatingRowColors(True)
        self._table.verticalHeader().setVisible(False)
        hdr = self._table.horizontalHeader()
        hdr.setSectionResizeMode(_COL_NAME,    QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(_COL_CREATED, QHeaderView.ResizeMode.ResizeToContents)
        self._table.itemDoubleClicked.connect(self._on_load)
        return self._table

    def _build_bottom_toolbar(self) -> QWidget:
        bar    = QWidget()
        row    = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)

        self._btn_new = QPushButton("New")
        self._btn_new.setToolTip("Create a new profile")
        self._btn_new.clicked.connect(self._on_new)
        row.addWidget(self._btn_new)

        self._btn_duplicate = QPushButton("Duplicate")
        self._btn_duplicate.setToolTip("Duplicate the selected profile")
        self._btn_duplicate.setEnabled(False)
        self._btn_duplicate.clicked.connect(self._on_duplicate)
        row.addWidget(self._btn_duplicate)

        self._btn_rename = QPushButton("Rename")
        self._btn_rename.setToolTip("Rename the selected profile")
        self._btn_rename.setEnabled(False)
        self._btn_rename.clicked.connect(self._on_rename)
        row.addWidget(self._btn_rename)

        self._btn_delete = QPushButton("Delete")
        self._btn_delete.setToolTip("Delete the selected profile from disk")
        self._btn_delete.setEnabled(False)
        self._btn_delete.clicked.connect(self._on_delete)
        row.addWidget(self._btn_delete)

        row.addStretch()

        self._btn_load = QPushButton("Load")
        self._btn_load.setToolTip("Send the selected profile to the run controls")
        self._btn_load.setEnabled(False)
        self._btn_load.clicked.connect(self._on_load)
        row.addWidget(self._btn_load)

        self._table.itemSelectionChanged.connect(self._on_selection_changed)

        return bar

    # ------------------------------------------------------------------
    # Private — table population
    # ------------------------------------------------------------------

    def _populate_table(self) -> None:
        self._table.setRowCount(0)
        for row_idx, p in enumerate(self._profiles):
            self._table.insertRow(row_idx)
            self._table.setItem(row_idx, _COL_NAME,    _cell(p.name))
            self._table.setItem(row_idx, _COL_DIP,     _cell(f"{p.dip_speed_mm_s:.1f}"))
            self._table.setItem(row_idx, _COL_WDRAW,   _cell(f"{p.withdraw_speed_mm_s:.1f}"))
            self._table.setItem(row_idx, _COL_ACCEL,   _cell(f"{p.accel_mm_s2:.1f}"))
            self._table.setItem(row_idx, _COL_DEPTH,   _cell(f"{p.dip_depth_mm:.1f}"))
            self._table.setItem(row_idx, _COL_DIPS,    _cell(str(p.n_dips)))
            # Show only the date portion of the ISO timestamp for readability.
            created = p.created_at[:10] if p.created_at else "—"
            self._table.setItem(row_idx, _COL_CREATED, _cell(created))
        self._on_selection_changed()
        self._lbl_dir.setText(str(self._profile_dir))

    # ------------------------------------------------------------------
    # Private — helpers
    # ------------------------------------------------------------------

    def _selected_row(self) -> int:
        """Return the currently selected row index, or -1 if none."""
        rows = self._table.selectedItems()
        if not rows:
            return -1
        return self._table.row(rows[0])

    def _selected_profile(self) -> Optional[DipProfile]:
        row = self._selected_row()
        if row < 0 or row >= len(self._profiles):
            return None
        return self._profiles[row]

    def _profile_path(self, profile: DipProfile) -> Path:
        """Return the expected JSON path for a profile."""
        safe = profile.name.strip().replace(" ", "_")
        return self._profile_dir / f"{safe}.json"

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_selection_changed(self) -> None:
        has_sel = self._selected_row() >= 0
        self._btn_duplicate.setEnabled(has_sel)
        self._btn_rename.setEnabled(has_sel)
        self._btn_delete.setEnabled(has_sel)
        self._btn_load.setEnabled(has_sel)

    def _on_new(self) -> None:
        dlg = _NewProfileDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            profile = dlg.get_profile()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid profile", str(exc))
            return
        path = self._profile_path(profile)
        if path.exists():
            reply = QMessageBox.question(
                self, "File exists",
                f"{path.name} already exists.  Overwrite?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        try:
            save_profile(profile, path)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self.refresh()
        log.info("new profile saved: %s", path)

    def _on_duplicate(self) -> None:
        profile = self._selected_profile()
        if profile is None:
            return
        new_name, ok = _ask_name(self, "Duplicate profile", f"{profile.name}_copy")
        if not ok or not new_name:
            return
        import dataclasses
        new_profile = dataclasses.replace(profile, name=new_name, created_at="")
        # clear created_at so __post_init__ stamps fresh time
        path = self._profile_path(new_profile)
        if path.exists():
            QMessageBox.warning(self, "Name in use",
                                f"{path.name} already exists.  Choose a different name.")
            return
        try:
            save_profile(new_profile, path)
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self.refresh()

    def _on_rename(self) -> None:
        profile = self._selected_profile()
        if profile is None:
            return
        old_path = self._profile_path(profile)
        new_name, ok = _ask_name(self, "Rename profile", profile.name)
        if not ok or not new_name or new_name == profile.name:
            return
        import dataclasses
        renamed = dataclasses.replace(profile, name=new_name)
        new_path = self._profile_path(renamed)
        if new_path.exists():
            QMessageBox.warning(self, "Name in use",
                                f"{new_path.name} already exists.  Choose a different name.")
            return
        try:
            save_profile(renamed, new_path)
            if old_path.exists():
                old_path.unlink()
        except OSError as exc:
            QMessageBox.critical(self, "Rename failed", str(exc))
            return
        self.refresh()

    def _on_delete(self) -> None:
        profile = self._selected_profile()
        if profile is None:
            return
        path = self._profile_path(profile)
        reply = QMessageBox.question(
            self, "Delete profile",
            f"Delete '{profile.name}'?\n{path}\n\nThis cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        try:
            if path.exists():
                path.unlink()
        except OSError as exc:
            QMessageBox.critical(self, "Delete failed", str(exc))
            return
        self.refresh()
        log.info("profile deleted: %s", path)

    def _on_load(self) -> None:
        profile = self._selected_profile()
        if profile is None:
            return
        self.profile_selected.emit(profile)
        log.info("profile loaded: %s", profile.name)


# ---------------------------------------------------------------------------
# Module helpers
# ---------------------------------------------------------------------------

def _cell(text: str) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
    return item


def _ask_name(parent: QWidget, title: str, default: str) -> tuple[str, bool]:
    """Show a simple single-line input dialog, return (text, accepted)."""
    from PyQt6.QtWidgets import QInputDialog
    text, ok = QInputDialog.getText(parent, title, "Profile name:", text=default)
    return text.strip(), ok
