"""
ui/tabs/serial_monitor_tab.py
=============================

Serial monitor tab — 2×2 grid of log panels + command input.

Layout
------

    ┌──────────────────────────┬────────────────────────────┐
    │  TX  (sent commands)     │  Sent History              │
    │  amber text              │  click to reload           │
    │                          │                            │
    ├──────────────────────────┼────────────────────────────┤
    │  TELEM  (RX telem lines) │  Other RX  (ACK/ERR/…)    │
    │  grey text               │  green/red/white           │
    │                          │                            │
    └──────────────────────────┴────────────────────────────┘
    ┌────────────────────────────────────────────────────────┐
    │  Quick: [dropdown]  [Send]                             │
    │  ┌─ Command Input (Ctrl+Enter to send) ──────────────┐ │
    │  │                                                   │ │
    │  └───────────────────────────────────────────────────┘ │
    │  [✓ Auto-scroll]                          [Clear]      │
    └────────────────────────────────────────────────────────┘

Grid cells:
  (0,0) TX commands sent to Arduino
  (0,1) Sent command history sidebar
  (1,0) TELEM lines received from Arduino
  (1,1) All other RX lines (ACK, ERR, STATE, DIAG, …)

Public API (called by ``MainWindow``):

    tab.set_manager(mgr)   # pass SerialManager / MockArduino (or None)

The tab polls ``mgr.raw_queue`` at 20 Hz via a ``QTimer``.
"""

from __future__ import annotations

import logging
import queue
from typing import Callable, Final, Optional

from PyQt6.QtCore import Qt, QRect, QStringListModel, QTimer
from PyQt6.QtGui import QBrush, QColor, QTextCursor, QKeyEvent
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QCompleter, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QPlainTextEdit,
    QGridLayout, QVBoxLayout, QWidget,
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
    "CMD GET_STATE",
    "CMD JOG UP",
    "CMD JOG DOWN",
    "CMD JOG UP 5.0",
    "CMD JOG DOWN 5.0",
    "CMD SET_TELEM_RATE",
    "CMD SET_SOFT_LIMITS",
    "CMD RUN_PROFILE",
    "CMD BEGIN_SEGMENTED_MOVE",
    "CMD MOVE_SEG",
    "CMD DWELL_SEG",
    "CMD RUN_LOADED_MOVE",
]

# Quick-command presets shown in the dropdown
_QUICK_COMMANDS: Final[list[str]] = [
    "CMD GET_STATE",
    "CMD HOME",
    "CMD STOP",
    "CMD ESTOP",
    "CMD JOG UP 5.0",
    "CMD JOG DOWN 5.0",
]

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------

_COL_TX:    Final[str] = "#e6a817"   # amber
_COL_ACK:   Final[str] = "#4caf50"   # green
_COL_ERR:   Final[str] = "#ef5350"   # red
_COL_TELEM: Final[str] = "#888888"   # grey
_COL_OTHER: Final[str] = "#eeeeee"   # white

# Maximum items per list widget before oldest entries are trimmed.
_MAX_LIST_ITEMS: Final[int] = 2000

_LIST_STYLE: Final[str] = (
    "QListWidget {"
    "  background-color: #1e1e1e;"
    "  color: #eeeeee;"
    "  font-family: Consolas, 'Courier New', monospace;"
    "  font-size: 9pt;"
    "  border: none;"
    "}"
)


def _other_rx_color(line: str) -> str:
    if line.startswith("RX ACK"):
        return _COL_ACK
    if line.startswith("RX ERR") or line.startswith("ERR"):
        return _COL_ERR
    return _COL_OTHER


# ---------------------------------------------------------------------------
# _CommandInput — QPlainTextEdit with autocomplete + history navigation
# ---------------------------------------------------------------------------

class _CommandInput(QPlainTextEdit):
    """Multi-line command input with autocomplete, history nav, and Ctrl+Enter send."""

    def __init__(self, send_callback: Callable[[], None], parent=None) -> None:
        super().__init__(parent)
        self._send_cb:  Callable[[], None] = send_callback
        self._history:  list[str]          = []
        self._hist_idx: int                = -1

        self.setPlaceholderText(
            "Type command(s), one per line.\n"
            "Ctrl+Enter to send  |  Tab to autocomplete  |  ↑/↓ for history"
        )
        self.setFixedHeight(90)
        self.setFont(self.document().defaultFont())

        self._model:     QStringListModel = QStringListModel(KNOWN_COMMANDS)
        self._completer: QCompleter       = QCompleter(self._model, self)
        self._completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        self._completer.setFilterMode(Qt.MatchFlag.MatchContains)
        self._completer.setWidget(self)
        self._completer.activated.connect(self._insert_completion)

    def push_history(self, batch: str) -> None:
        """Insert a sent command batch at the top of the history stack.

        Blank batches are discarded.  The history index is reset to ``-1``
        so the next Up-arrow press retrieves the most recently sent entry.

        Args:
            batch: One or more commands separated by newlines.
        """
        if batch.strip():
            self._history.insert(0, batch)
        self._hist_idx = -1

    def keyPressEvent(self, event: QKeyEvent) -> None:  # type: ignore[override]
        """Handle keyboard shortcuts for sending, autocomplete, history navigation, and dismiss.

        Key bindings:

        * **Ctrl+Enter** — send all lines in the input area.
        * **Tab** (popup visible) — accept the highlighted autocomplete suggestion.
        * **Up / Down** (popup hidden) — scroll through the sent-command history.
        * **Escape** — dismiss the autocomplete popup.
        * All other keys — passed to the base class, then the autocomplete popup is
          refreshed to reflect any text change.
        """
        # Ctrl+Enter — send immediately without inserting a newline.
        if (
            event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter)
            and event.modifiers() & Qt.KeyboardModifier.ControlModifier
        ):
            self._send_cb()
            return

        # Tab — accept the currently highlighted autocomplete suggestion.
        if event.key() == Qt.Key.Key_Tab and self._completer.popup().isVisible():
            self._completer.popup().hide()
            idx = self._completer.popup().currentIndex()
            if idx.isValid():
                self._insert_completion(self._completer.completionModel().data(idx))
            else:
                self._completer.setCurrentRow(0)
                self._insert_completion(self._completer.currentCompletion())
            return

        # Up arrow — navigate backward through history (when popup is not showing).
        if event.key() == Qt.Key.Key_Up and not self._completer.popup().isVisible():
            if self._history and self._hist_idx < len(self._history) - 1:
                self._hist_idx += 1
                self.setPlainText(self._history[self._hist_idx])
                self._move_cursor_end()
            return

        # Down arrow — navigate forward through history; clear input at the end.
        if event.key() == Qt.Key.Key_Down and not self._completer.popup().isVisible():
            if self._hist_idx > 0:
                self._hist_idx -= 1
                self.setPlainText(self._history[self._hist_idx])
                self._move_cursor_end()
            elif self._hist_idx == 0:
                self._hist_idx = -1
                self.clear()
            return

        # Escape — dismiss the autocomplete popup without inserting anything.
        if event.key() == Qt.Key.Key_Escape:
            self._completer.popup().hide()
            return

        # All other keys — default handling, then refresh the autocomplete popup.
        super().keyPressEvent(event)
        self._update_completer()

    def _current_line_text(self) -> str:
        """Return the full text of the line that currently contains the cursor.

        Returns:
            The raw text of the current line, including any leading whitespace.
        """
        cursor: QTextCursor = self.textCursor()
        cursor.select(QTextCursor.SelectionType.LineUnderCursor)
        return cursor.selectedText()

    def _insert_completion(self, completion: str) -> None:
        """Replace the entire current line with the selected autocomplete suggestion.

        Args:
            completion: The fully completed command string to insert.
        """
        cursor: QTextCursor = self.textCursor()
        cursor.select(QTextCursor.SelectionType.LineUnderCursor)
        cursor.insertText(completion)
        self.setTextCursor(cursor)
        self._completer.popup().hide()

    def _move_cursor_end(self) -> None:
        """Move the text cursor to the very end of the document."""
        cursor: QTextCursor = self.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self.setTextCursor(cursor)

    def _update_completer(self) -> None:
        """Refresh and reposition the autocomplete popup for the text on the current line.

        The popup is hidden when the prefix is shorter than 3 characters or when
        no known command matches the prefix.  The popup is sized to fit the widest
        suggestion plus a vertical scrollbar allowance.
        """
        # Use the stripped current line as the completion prefix.
        prefix: str = self._current_line_text().lstrip()

        # Require at least 3 characters before showing suggestions.
        if len(prefix) < 3:
            self._completer.popup().hide()
            return

        self._completer.setCompletionPrefix(prefix)
        if self._completer.completionCount() == 0:
            self._completer.popup().hide()
            return

        # Size and position the popup directly below the cursor.
        rect: QRect = self.cursorRect()
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
    Serial monitor tab — 2×2 grid of log panels.

    (0,0) TX sent   (0,1) Sent History
    (1,0) TELEM RX  (1,1) Other RX (ACK/ERR/…)

    Call :meth:`set_manager` whenever the connection changes.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._manager = None
        self._auto_scroll: bool = True

        self._build_ui()

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
        """Assemble the 2×2 log grid, quick-command row, command input box, and toolbar."""
        outer = QVBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.setSpacing(4)

        # ---- 2×2 log grid ------------------------------------------------
        grid = QGridLayout()
        grid.setSpacing(4)

        self._tx_list      = self._make_list()
        self._history_list = self._make_list(selectable=True)
        self._telem_list   = self._make_list()
        self._rx_list      = self._make_list()

        self._history_list.setToolTip("Click a row to reload it in the input area")
        self._history_list.setStyleSheet(
            "QListWidget {"
            "  font-family: Consolas, 'Courier New', monospace;"
            "  font-size: 9pt;"
            "}"
        )
        self._history_list.itemClicked.connect(self._on_history_clicked)

        tx_grp    = self._wrap_group("TX  (sent)",            self._tx_list)
        hist_grp  = self._wrap_group("Sent History",          self._history_list)
        telem_grp = self._wrap_group("TELEM  (received)",     self._telem_list)
        rx_grp    = self._wrap_group("Other RX  (ACK / ERR)", self._rx_list)

        grid.addWidget(tx_grp,    0, 0)
        grid.addWidget(hist_grp,  0, 1)
        grid.addWidget(telem_grp, 1, 0)
        grid.addWidget(rx_grp,    1, 1)

        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        outer.addLayout(grid, stretch=1)

        # ---- Quick-command row -------------------------------------------
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
        outer.addLayout(quick_row)

        # ---- Command input -----------------------------------------------
        input_grp = QGroupBox("Command Input  (Ctrl+Enter to send)")
        input_layout = QVBoxLayout(input_grp)
        input_layout.setContentsMargins(4, 4, 4, 4)
        self._input = _CommandInput(send_callback=self._on_send)
        input_layout.addWidget(self._input)
        outer.addWidget(input_grp)

        # ---- Toolbar row -------------------------------------------------
        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)
        self._chk_autoscroll = QCheckBox("Auto-scroll")
        self._chk_autoscroll.setChecked(True)
        self._chk_autoscroll.toggled.connect(self._on_autoscroll_toggled)
        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(60)
        btn_clear.clicked.connect(self._clear_logs)
        toolbar.addWidget(self._chk_autoscroll)
        toolbar.addStretch()
        toolbar.addWidget(btn_clear)
        outer.addLayout(toolbar)

    @staticmethod
    def _make_list(selectable: bool = False) -> QListWidget:
        """Create a monospace-font styled ``QListWidget``.

        Args:
            selectable: When ``True``, rows can be clicked to select them
                        (used for the Sent History sidebar).  When ``False``
                        (the default), selection is disabled so the list acts
                        as a pure read-only log.

        Returns:
            A configured ``QListWidget`` ready to be added to a layout.
        """
        lst = QListWidget()
        lst.setStyleSheet(_LIST_STYLE)
        if not selectable:
            lst.setSelectionMode(QListWidget.SelectionMode.NoSelection)
        return lst

    @staticmethod
    def _wrap_group(title: str, widget: QWidget) -> QGroupBox:
        """Wrap *widget* inside a titled ``QGroupBox`` with tight margins.

        Args:
            title:  Text displayed in the group box border.
            widget: The widget to place inside the group box.

        Returns:
            A ``QGroupBox`` containing *widget*.
        """
        grp = QGroupBox(title)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(2, 4, 2, 2)
        lay.addWidget(widget)
        return grp

    # ------------------------------------------------------------------
    # Private — slot handlers
    # ------------------------------------------------------------------

    def _on_send(self) -> None:
        """Send every non-empty line from the input area; log each line to TX and history.

        Lines are sent one at a time via ``manager.send_command``.  All lines are
        joined into a single history entry so the operator can reload the full batch
        with one click on the Sent History sidebar.  The input area is cleared after
        a successful send.
        """
        if self._manager is None:
            return

        # Collect and strip all non-blank lines from the multi-line input.
        raw_text: str       = self._input.toPlainText()
        lines:    list[str] = [ln.strip() for ln in raw_text.splitlines() if ln.strip()]
        if not lines:
            return

        # Send each command line individually; log errors to the RX panel.
        line: str
        for line in lines:
            try:
                self._manager.send_command(line)
            except Exception as exc:
                self._append_to(self._rx_list, f"ERR {exc}", _COL_ERR)
                log.error("send_command(%r) failed: %s", line, exc)

        # Push the full batch to history and clear the input area.
        batch: str = "\n".join(lines)
        self._input.push_history(batch)
        self._input.clear()

        # Add a preview entry to the Sent History sidebar (truncated if long).
        preview: str             = batch if len(batch) <= 60 else batch[:57] + "…"
        item:    QListWidgetItem = QListWidgetItem(preview)
        item.setData(Qt.ItemDataRole.UserRole, batch)  # store the full batch for reloading
        self._history_list.insertItem(0, item)

    def _on_quick_send(self) -> None:
        """Send the command currently shown in the quick-command dropdown."""
        cmd: str = self._combo_quick.currentText().strip()
        if not cmd or self._manager is None:
            return
        try:
            self._manager.send_command(cmd)
        except Exception as exc:
            self._append_to(self._rx_list, f"ERR {exc}", _COL_ERR)

    def _on_history_clicked(self, item: QListWidgetItem) -> None:
        """Load a previously sent batch back into the command input area.

        The full multi-line batch is retrieved from the item's ``UserRole`` data
        (the visible text may be truncated to 60 characters).  Focus is moved to
        the input so the operator can immediately edit or resend.

        Args:
            item: The history sidebar row that was clicked.
        """
        batch: str = item.data(Qt.ItemDataRole.UserRole)
        self._input.setPlainText(batch)
        cursor = self._input.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        self._input.setTextCursor(cursor)
        self._input.setFocus()

    def _on_autoscroll_toggled(self, checked: bool) -> None:
        """Enable or disable automatic scrolling to the bottom of all log lists.

        Args:
            checked: ``True`` to enable auto-scroll; ``False`` to lock the scroll position.
        """
        self._auto_scroll = checked

    def _clear_logs(self) -> None:
        """Clear the TX sent, TELEM received, and Other RX log lists.

        The Sent History sidebar is intentionally preserved so the operator can
        still reload and resend commands after clearing the live log view.
        """
        self._tx_list.clear()
        self._telem_list.clear()
        self._rx_list.clear()

    # ------------------------------------------------------------------
    # Private — raw_queue polling
    # ------------------------------------------------------------------

    def _poll_raw_queue(self) -> None:
        """Drain ``manager.raw_queue`` and route each line to the correct log list.

        Called by a 20 Hz ``QTimer``.  At most 50 lines are processed per tick to
        prevent the UI from stalling during bursts of rapid serial output.

        Routing rules:

        * Lines starting with ``"TX "`` → TX sent panel (amber).
        * Lines starting with ``"RX TELEM"`` → TELEM received panel (grey).
        * All other lines → Other RX panel (colour determined by :func:`_other_rx_color`).
        """
        if self._manager is None:
            return

        raw_q: Optional[queue.Queue[str]] = getattr(self._manager, "raw_queue", None)
        if raw_q is None:
            return

        # Drain up to 50 lines per timer tick.
        count: int = 0
        while count < 50:
            try:
                line: str = raw_q.get_nowait()
            except Exception:
                break

            # Route to the appropriate log list based on the line prefix.
            if line.startswith("TX "):
                self._append_to(self._tx_list, line, _COL_TX)
            elif line.startswith("RX TELEM"):
                self._append_to(self._telem_list, line, _COL_TELEM)
            else:
                self._append_to(self._rx_list, line, _other_rx_color(line))
            count += 1

    # ------------------------------------------------------------------
    # Private — list append helper
    # ------------------------------------------------------------------

    def _append_to(self, lst: QListWidget, text: str, color: str) -> None:
        """Append a coloured text item to *lst*, trimming the oldest entry when full.

        The list is capped at :data:`_MAX_LIST_ITEMS` rows so memory usage stays
        bounded during long sessions.  If auto-scroll is enabled, the view scrolls
        to show the newly added item.

        Args:
            lst:   The target ``QListWidget`` (TX, TELEM, or RX panel).
            text:  The raw log line to display.
            color: CSS foreground colour string for the item text.
        """
        item = QListWidgetItem(text)
        item.setForeground(QBrush(QColor(color)))
        lst.addItem(item)

        # Remove the oldest entry to keep the list within the size cap.
        while lst.count() > _MAX_LIST_ITEMS:
            lst.takeItem(0)

        if self._auto_scroll:
            lst.scrollToBottom()
