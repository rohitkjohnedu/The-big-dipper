"""
ui/tabs/serial_monitor_tab.py
=============================

Serial monitor tab — mirrors the Arduino IDE serial monitor with extras.

Layout (QSplitter — left pane wider by default)
------------------------------------------------
Left pane:
  * **Log view** — ``QPlainTextEdit`` (read-only), colour-coded lines::

        TX  →  amber  (#e6a817)
        ACK →  green  (#4caf50)
        ERR →  red    (#ef5350)
        TELEM → grey  (#888888)
        other →  white (#eeeeee)

  * **Quick-command bar** — ``QComboBox`` of common commands + Send button.
  * **Input area** — ``_CommandInput`` (``QPlainTextEdit`` subclass):
      - Tab / popup after 3 chars: ``QCompleter`` autocomplete (case-insensitive).
      - Up / Down arrows while empty: cycle through command history batches.
      - Ctrl+Enter: send all lines sequentially.
  * **Toolbar row** — Auto-scroll toggle, Clear button.

Right pane:
  * **History list** — ``QListWidget`` showing all previously sent batches
    (most-recent on top).  Clicking a row re-loads it into the input area.

Public API (called by ``MainWindow``):

    tab.set_manager(mgr)   # pass SerialManager / MockArduino (or None)

The tab polls ``mgr.raw_queue`` at 20 Hz via a ``QTimer``.

Command vocabulary
------------------
``KNOWN_COMMANDS`` is the autocomplete source.  It covers every command the
Arduino firmware accepts as of config.h / protocol.md.
"""

from __future__ import annotations

import logging
from typing import Final, Optional

from PyQt6.QtCore import Qt, QStringListModel, QTimer
from PyQt6.QtGui import QColor, QTextCharFormat, QTextCursor, QKeyEvent
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QCompleter, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QPlainTextEdit,
    QSplitter, QVBoxLayout, QWidget,
)

log: Final[logging.Logger] = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Known commands — source for autocomplete
# ---------------------------------------------------------------------------

KNOWN_COMMANDS: Final[list[str]] = [
    "CMD HOME",
    "CMD STOP",
    "CMD ESTOP",
    "CMD PAUSE",
    "CMD RESUME",
    "CMD JOG UP",
    "CMD JOG DOWN",
    "CMD JOG UP 5.0",
    "CMD JOG DOWN 5.0",
    "CMD MOVE_TO",
    "CMD SET_SPEED",
    "CMD RUN_PROFILE",
    "CMD STATUS",
    "CMD PING",
]

# Quick-command presets shown in the dropdown
_QUICK_COMMANDS: Final[list[str]] = [
    "CMD PING",
    "CMD STATUS",
    "CMD HOME",
    "CMD STOP",
    "CMD ESTOP",
    "CMD JOG UP 5.0",
    "CMD JOG DOWN 5.0",
]

# ---------------------------------------------------------------------------
# Log-line colours
# ---------------------------------------------------------------------------

_COL_TX:    Final[str] = "#e6a817"   # amber
_COL_ACK:   Final[str] = "#4caf50"   # green
_COL_ERR:   Final[str] = "#ef5350"   # red
_COL_TELEM: Final[str] = "#888888"   # grey
_COL_OTHER: Final[str] = "#eeeeee"   # white


def _color_for_line(line: str) -> str:
    if line.startswith("TX "):
        return _COL_TX
    if line.startswith("RX ACK"):
        return _COL_ACK
    if line.startswith("RX ERR"):
        return _COL_ERR
    if line.startswith("RX TELEM"):
        return _COL_TELEM
    return _COL_OTHER


# ---------------------------------------------------------------------------
# _CommandInput — QPlainTextEdit with autocomplete + history navigation
# ---------------------------------------------------------------------------

class _CommandInput(QPlainTextEdit):
    """
    Multi-line command input with:

    * **Autocomplete** — ``QCompleter`` (case-insensitive) attached to the
      current line's text; popup appears after 3 characters, Tab accepts.
    * **History navigation** — Up / Down arrows (when text is absent from
      the cursor line or only whitespace before cursor) cycle through
      previously sent batches loaded from ``_history``.
    * **Send trigger** — Ctrl+Enter calls the ``send_callback`` supplied at
      construction time.
    """

    def __init__(self, send_callback, parent=None) -> None:
        super().__init__(parent)
        self._send_cb = send_callback
        self._history: list[str] = []   # each element is a multi-line batch
        self._hist_idx: int = -1        # -1 = current draft

        self.setPlaceholderText(
            "Type command(s), one per line.\n"
            "Ctrl+Enter to send  |  Tab to autocomplete  |  ↑/↓ for history"
        )
        self.setFixedHeight(90)
        self.setFont(self.document().defaultFont())

        # --- QCompleter ---
        self._model = QStringListModel(KNOWN_COMMANDS)
        self._completer = QCompleter(self._model, self)
        self._completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self._completer.setFilterMode(Qt.MatchFlag.MatchContains)
        self._completer.setWidget(self)
        self._completer.activated.connect(self._insert_completion)

    # ------------------------------------------------------------------
    # Public helpers called by SerialMonitorTab
    # ------------------------------------------------------------------

    def push_history(self, batch: str) -> None:
        """Prepend a sent batch to the history list."""
        if batch.strip():
            self._history.insert(0, batch)
        self._hist_idx = -1

    # ------------------------------------------------------------------
    # Key handling
    # ------------------------------------------------------------------

    def keyPressEvent(self, event: QKeyEvent) -> None:  # type: ignore[override]
        # --- Ctrl+Enter → send --------------------------------------------
        if (
            event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter)
            and event.modifiers() & Qt.KeyboardModifier.ControlModifier
        ):
            self._send_cb()
            return

        # --- Tab → accept autocomplete suggestion -------------------------
        if event.key() == Qt.Key.Key_Tab and self._completer.popup().isVisible():
            self._completer.popup().hide()
            idx = self._completer.popup().currentIndex()
            if idx.isValid():
                self._insert_completion(self._completer.completionModel().data(idx))
            else:
                # Accept first match
                self._completer.setCurrentRow(0)
                self._insert_completion(self._completer.currentCompletion())
            return

        # --- Up arrow → history older -------------------------------------
        if event.key() == Qt.Key.Key_Up and not self._completer.popup().isVisible():
            if self._history and self._hist_idx < len(self._history) - 1:
                self._hist_idx += 1
                self.setPlainText(self._history[self._hist_idx])
                self._move_cursor_end()
            return

        # --- Down arrow → history newer -----------------------------------
        if event.key() == Qt.Key.Key_Down and not self._completer.popup().isVisible():
            if self._hist_idx > 0:
                self._hist_idx -= 1
                self.setPlainText(self._history[self._hist_idx])
                self._move_cursor_end()
            elif self._hist_idx == 0:
                self._hist_idx = -1
                self.clear()
            return

        # --- Escape → hide popup ------------------------------------------
        if event.key() == Qt.Key.Key_Escape:
            self._completer.popup().hide()
            return

        # Default handling
        super().keyPressEvent(event)

        # After every other keypress: update autocomplete
        self._update_completer()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _current_line_text(self) -> str:
        cursor = self.textCursor()
        cursor.select(QTextCursor.SelectionType.LineUnderCursor)
        return cursor.selectedText()

    def _insert_completion(self, completion: str) -> None:
        cursor = self.textCursor()
        cursor.select(QTextCursor.SelectionType.LineUnderCursor)
        cursor.insertText(completion)
        self.setTextCursor(cursor)
        self._completer.popup().hide()

    def _move_cursor_end(self) -> None:
        cursor = self.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.setTextCursor(cursor)

    def _update_completer(self) -> None:
        prefix = self._current_line_text().lstrip()
        if len(prefix) < 3:
            self._completer.popup().hide()
            return

        self._completer.setCompletionPrefix(prefix)
        if self._completer.completionCount() == 0:
            self._completer.popup().hide()
            return

        # Position the popup below the current line
        rect = self.cursorRect()
        rect.setWidth(
            self._completer.popup().sizeHintForColumn(0)
            + self._completer.popup().verticalScrollBar().sizeHint().width()
        )
        self._completer.complete(rect)


# ---------------------------------------------------------------------------
# SerialMonitorTab
# ---------------------------------------------------------------------------

class SerialMonitorTab(QWidget):
    """
    Serial monitor tab — real-time TX/RX log + multi-line command input.

    Call :meth:`set_manager` whenever the connection changes.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._manager = None
        self._auto_scroll: bool = True

        self._build_ui()

        # Poll raw_queue at 20 Hz
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(50)
        self._poll_timer.timeout.connect(self._poll_raw_queue)
        self._poll_timer.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_manager(self, manager) -> None:
        """Attach a ``SerialManager`` or ``MockArduino`` (or ``None`` to detach)."""
        self._manager = manager

    # ------------------------------------------------------------------
    # Private — UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        # ---- Left pane ---------------------------------------------------
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        # Log view
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(2000)
        self._log.setStyleSheet(
            "QPlainTextEdit {"
            "  background-color: #1e1e1e;"
            "  color: #eeeeee;"
            "  font-family: Consolas, 'Courier New', monospace;"
            "  font-size: 9pt;"
            "}"
        )
        left_layout.addWidget(self._log, stretch=1)

        # Quick-command row
        quick_row = QHBoxLayout()
        quick_row.setSpacing(4)
        self._combo_quick = QComboBox()
        self._combo_quick.addItems(_QUICK_COMMANDS)
        self._combo_quick.setEditable(False)
        self._combo_quick.setToolTip("Pick a common command and click Send")
        btn_quick_send = QPushButton("Send")
        btn_quick_send.setFixedWidth(60)
        btn_quick_send.clicked.connect(self._on_quick_send)
        quick_row.addWidget(QLabel("Quick:"))
        quick_row.addWidget(self._combo_quick, stretch=1)
        quick_row.addWidget(btn_quick_send)
        left_layout.addLayout(quick_row)

        # Command input
        input_grp = QGroupBox("Command Input  (Ctrl+Enter to send)")
        input_layout = QVBoxLayout(input_grp)
        input_layout.setContentsMargins(4, 4, 4, 4)
        self._input = _CommandInput(send_callback=self._on_send)
        input_layout.addWidget(self._input)
        left_layout.addWidget(input_grp)

        # Toolbar row
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        self._chk_autoscroll = QCheckBox("Auto-scroll")
        self._chk_autoscroll.setChecked(True)
        self._chk_autoscroll.toggled.connect(self._on_autoscroll_toggled)
        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(60)
        btn_clear.clicked.connect(self._log.clear)
        toolbar.addWidget(self._chk_autoscroll)
        toolbar.addStretch()
        toolbar.addWidget(btn_clear)
        left_layout.addLayout(toolbar)

        # ---- Right pane — history ----------------------------------------
        right_grp = QGroupBox("Sent History")
        right_layout = QVBoxLayout(right_grp)
        right_layout.setContentsMargins(4, 4, 4, 4)
        self._history_list = QListWidget()
        self._history_list.setToolTip("Click a row to reload it in the input area")
        self._history_list.setStyleSheet(
            "QListWidget {"
            "  font-family: Consolas, 'Courier New', monospace;"
            "  font-size: 9pt;"
            "}"
        )
        self._history_list.itemClicked.connect(self._on_history_clicked)
        right_layout.addWidget(self._history_list)
        right_grp.setMinimumWidth(180)

        splitter.addWidget(left)
        splitter.addWidget(right_grp)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        outer.addWidget(splitter)

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_send(self) -> None:
        """Send all lines in the input area sequentially."""
        if self._manager is None:
            return
        raw_text = self._input.toPlainText()
        lines = [ln.strip() for ln in raw_text.splitlines() if ln.strip()]
        if not lines:
            return

        for line in lines:
            try:
                self._manager.send_command(line)
            except Exception as exc:
                self._append_line(f"ERR {exc}", _COL_ERR)
                log.error("send_command(%r) failed: %s", line, exc)

        # Push to history (multi-line batch stored as-is)
        batch = "\n".join(lines)
        self._input.push_history(batch)
        self._input.clear()

        # Add to history sidebar (most recent at top)
        preview = batch if len(batch) <= 60 else batch[:57] + "…"
        item = QListWidgetItem(preview)
        item.setData(Qt.ItemDataRole.UserRole, batch)
        self._history_list.insertItem(0, item)

    def _on_quick_send(self) -> None:
        cmd = self._combo_quick.currentText().strip()
        if not cmd or self._manager is None:
            return
        try:
            self._manager.send_command(cmd)
        except Exception as exc:
            self._append_line(f"ERR {exc}", _COL_ERR)

    def _on_history_clicked(self, item: QListWidgetItem) -> None:
        batch: str = item.data(Qt.ItemDataRole.UserRole)
        self._input.setPlainText(batch)
        cursor = self._input.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self._input.setTextCursor(cursor)
        self._input.setFocus()

    def _on_autoscroll_toggled(self, checked: bool) -> None:
        self._auto_scroll = checked

    # ------------------------------------------------------------------
    # Private — raw_queue polling
    # ------------------------------------------------------------------

    def _poll_raw_queue(self) -> None:
        if self._manager is None:
            return
        raw_q = getattr(self._manager, "raw_queue", None)
        if raw_q is None:
            return
        count = 0
        while count < 50:   # drain at most 50 lines per tick to stay responsive
            try:
                line: str = raw_q.get_nowait()
            except Exception:
                break
            self._append_line(line, _color_for_line(line))
            count += 1

    # ------------------------------------------------------------------
    # Private — log append
    # ------------------------------------------------------------------

    def _append_line(self, text: str, color: str) -> None:
        cursor = self._log.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)

        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        cursor.insertText(text + "\n")

        if self._auto_scroll:
            self._log.setTextCursor(cursor)
            self._log.ensureCursorVisible()
