"""
tests/ui/test_widgets.py
========================

pytest-qt tests for the three UI widgets:

* :class:`~ui.widgets.estop_button.EstopButton`
* :class:`~ui.widgets.live_plot.LivePlot`
* :class:`~ui.widgets.status_bar.StatusBar`

All tests run headless (no physical display required on CI) because
pytest-qt creates an offscreen QApplication automatically.
"""

from __future__ import annotations

import queue
import time
from unittest.mock import MagicMock, patch

import pytest
from pytestqt.qtbot import QtBot

from core.telemetry_parser import TelemetryFrame
from ui.widgets.estop_button import EstopButton
from ui.widgets.live_plot import LivePlot
from ui.widgets.status_bar import StatusBar


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_frame(
    state:    str   = "RUNNING",
    phase:    str   = "DESCENDING",
    pos_mm:   float = 0.0,
    vel_cmd:  float = 5.0,
    vel_act:  float = 4.8,
    accel:    float = 0.0,
    ts_ms:    int   = 1000,
) -> TelemetryFrame:
    return TelemetryFrame(
        timestamp_ms       = ts_ms,
        pos_mm             = pos_mm,
        vel_actual_mm_s    = vel_act,
        vel_commanded_mm_s = vel_cmd,
        accel_mm_s2        = accel,
        state              = state,
        phase              = phase,
    )


def _make_ci_mock() -> MagicMock:
    """Return a MagicMock that stands in for CommandInterface."""
    return MagicMock()


# ---------------------------------------------------------------------------
# EstopButton
# ---------------------------------------------------------------------------

class TestEstopButton:

    def test_button_visible_and_labelled(self, qtbot: QtBot) -> None:
        """Button is shown with the correct label text."""
        ci = _make_ci_mock()
        btn = EstopButton(command_interface=ci)
        qtbot.addWidget(btn)
        btn.show()
        assert btn.text() == "EMERGENCY STOP"
        assert btn.isVisible()

    def test_click_calls_estop(self, qtbot: QtBot) -> None:
        """Clicking the button calls CommandInterface.estop() exactly once."""
        ci = _make_ci_mock()
        btn = EstopButton(command_interface=ci)
        qtbot.addWidget(btn)
        btn.show()
        qtbot.mouseClick(btn, pytest.importorskip("PyQt6.QtCore").Qt.MouseButton.LeftButton)
        ci.estop.assert_called_once()

    def test_button_never_disabled(self, qtbot: QtBot) -> None:
        """The button is enabled by default and must remain enabled."""
        ci = _make_ci_mock()
        btn = EstopButton(command_interface=ci)
        qtbot.addWidget(btn)
        assert btn.isEnabled()

    def test_minimum_size(self, qtbot: QtBot) -> None:
        """Button meets the minimum tap-target dimensions."""
        ci = _make_ci_mock()
        btn = EstopButton(command_interface=ci)
        qtbot.addWidget(btn)
        assert btn.minimumWidth()  >= 160
        assert btn.minimumHeight() >= 64

    def test_clicked_signal_emitted(self, qtbot: QtBot) -> None:
        """The standard clicked signal fires (so callers can connect slots)."""
        ci = _make_ci_mock()
        btn = EstopButton(command_interface=ci)
        qtbot.addWidget(btn)
        btn.show()
        with qtbot.waitSignal(btn.clicked, timeout=1000):
            qtbot.mouseClick(btn, pytest.importorskip("PyQt6.QtCore").Qt.MouseButton.LeftButton)


# ---------------------------------------------------------------------------
# LivePlot
# ---------------------------------------------------------------------------

class TestLivePlot:

    def test_creates_without_error(self, qtbot: QtBot) -> None:
        """LivePlot constructs and shows without raising."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot.show()

    def test_push_single_frame(self, qtbot: QtBot) -> None:
        """push_frame() with one frame does not raise."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot.push_frame(_make_frame(ts_ms=0))

    def test_push_multiple_frames(self, qtbot: QtBot) -> None:
        """push_frame() accumulates data without error."""
        plot = LivePlot(history_s=10.0, telem_hz=10)
        qtbot.addWidget(plot)
        for i in range(50):
            plot.push_frame(_make_frame(ts_ms=i * 100, pos_mm=float(i)))

    def test_buffer_bounded_by_history(self, qtbot: QtBot) -> None:
        """Deque stays within maxlen — old frames are discarded."""
        history_s = 5.0
        telem_hz  = 10
        plot = LivePlot(history_s=history_s, telem_hz=telem_hz)
        qtbot.addWidget(plot)

        n_frames = int(history_s * telem_hz) + 50   # deliberately overfill
        for i in range(n_frames):
            plot.push_frame(_make_frame(ts_ms=i * 100))

        max_expected = int(history_s * telem_hz) + 1
        assert len(plot._t) <= max_expected

    def test_clear_resets_buffers(self, qtbot: QtBot) -> None:
        """clear() empties all data buffers and resets t0."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        for i in range(10):
            plot.push_frame(_make_frame(ts_ms=i * 100))
        plot.clear()
        assert len(plot._t) == 0
        assert plot._t0 is None

    def test_clear_then_push(self, qtbot: QtBot) -> None:
        """After clear(), new frames are accepted and t restarts from 0."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        for i in range(5):
            plot.push_frame(_make_frame(ts_ms=i * 100))
        plot.clear()
        plot.push_frame(_make_frame(ts_ms=9999))
        assert len(plot._t) == 1
        assert plot._t[0] == pytest.approx(0.0)

    def test_time_axis_starts_at_zero(self, qtbot: QtBot) -> None:
        """First pushed frame always maps to t=0 regardless of timestamp_ms."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot.push_frame(_make_frame(ts_ms=50000))
        assert plot._t[0] == pytest.approx(0.0)

    def test_time_axis_monotonic(self, qtbot: QtBot) -> None:
        """Time values increase with each successive frame."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        for i in range(5):
            plot.push_frame(_make_frame(ts_ms=i * 200))
        t = list(plot._t)
        assert all(t[i] < t[i + 1] for i in range(len(t) - 1))

    # ------------------------------------------------------------------
    # Home button
    # ------------------------------------------------------------------

    def test_home_button_exists(self, qtbot: QtBot) -> None:
        """Toolbar has a Home button."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        assert hasattr(plot, "_btn_window")   # toolbar was built

    def test_home_does_not_raise(self, qtbot: QtBot) -> None:
        """Calling _on_home() without data does not raise."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot._on_home()

    def test_home_after_frames_does_not_raise(self, qtbot: QtBot) -> None:
        """_on_home() after data is pushed does not raise."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        for i in range(5):
            plot.push_frame(_make_frame(ts_ms=i * 100))
        plot._on_home()

    # ------------------------------------------------------------------
    # Window / Full History mode
    # ------------------------------------------------------------------

    def test_window_mode_on_by_default(self, qtbot: QtBot) -> None:
        """Window mode button is checked, Full History is unchecked on init."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        assert plot._btn_window.isChecked()
        assert not plot._btn_full.isChecked()

    def test_switch_to_full_history_mode(self, qtbot: QtBot) -> None:
        """_on_full_history_mode() activates Full History and deactivates Window."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot._on_full_history_mode()
        assert plot._btn_full.isChecked()
        assert not plot._btn_window.isChecked()

    def test_switch_back_to_window_mode(self, qtbot: QtBot) -> None:
        """_on_window_mode() re-activates Window after Full History."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot._on_full_history_mode()
        plot._on_window_mode()
        assert plot._btn_window.isChecked()
        assert not plot._btn_full.isChecked()

    def test_full_history_retains_all_frames(self, qtbot: QtBot) -> None:
        """In Full History mode all pushed frames are stored in _all_t."""
        plot = LivePlot(history_s=2.0, telem_hz=10)
        qtbot.addWidget(plot)
        n = 50   # more than the 2 s rolling window holds
        for i in range(n):
            plot.push_frame(_make_frame(ts_ms=i * 100))
        assert len(plot._all_t) == n

    def test_full_history_cleared_by_clear(self, qtbot: QtBot) -> None:
        """clear() resets full-history buffers too."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        for i in range(10):
            plot.push_frame(_make_frame(ts_ms=i * 100))
        plot.clear()
        assert len(plot._all_t) == 0

    def test_spinbox_default_matches_history_s(self, qtbot: QtBot) -> None:
        """Window spinbox initialises to the history_s constructor argument."""
        plot = LivePlot(history_s=90.0)
        qtbot.addWidget(plot)
        assert plot._spin_window.value() == 90

    def test_spinbox_disabled_in_full_history_mode(self, qtbot: QtBot) -> None:
        """Window spinbox is disabled when Full History mode is active."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot._on_full_history_mode()
        assert not plot._spin_window.isEnabled()

    def test_spinbox_enabled_in_window_mode(self, qtbot: QtBot) -> None:
        """Window spinbox is enabled when Window mode is active."""
        plot = LivePlot()
        qtbot.addWidget(plot)
        plot._on_full_history_mode()
        plot._on_window_mode()
        assert plot._spin_window.isEnabled()


# ---------------------------------------------------------------------------
# StatusBar
# ---------------------------------------------------------------------------

class TestStatusBar:

    def test_creates_without_error(self, qtbot: QtBot) -> None:
        """StatusBar constructs and shows without raising."""
        bar = StatusBar(port="COM4")
        qtbot.addWidget(bar)
        bar.show()

    def test_initial_state_label(self, qtbot: QtBot) -> None:
        """State label shows IDLE on construction."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        assert bar._lbl_state.text() == "IDLE"

    def test_update_frame_state(self, qtbot: QtBot) -> None:
        """update_frame() updates the state label."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING", phase="DESCENDING"))
        assert bar._lbl_state.text() == "RUNNING"

    def test_update_frame_phase(self, qtbot: QtBot) -> None:
        """update_frame() updates the phase label (underscores replaced by spaces)."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING", phase="DWELL_BOTTOM"))
        assert bar._lbl_phase.text() == "DWELL BOTTOM"

    def test_set_connected_true(self, qtbot: QtBot) -> None:
        """set_connected(True) puts the port name in the port label."""
        bar = StatusBar(port="COM4")
        qtbot.addWidget(bar)
        bar.set_connected(True)
        assert "COM4" in bar._lbl_port.text()

    def test_set_connected_overrides_port_name(self, qtbot: QtBot) -> None:
        """set_connected(True, port='COM7') updates the displayed port name."""
        bar = StatusBar(port="COM4")
        qtbot.addWidget(bar)
        bar.set_connected(True, port="COM7")
        assert "COM7" in bar._lbl_port.text()

    def test_elapsed_resets_on_idle(self, qtbot: QtBot) -> None:
        """Transitioning to IDLE resets the elapsed label to 00:00.0."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING"))
        bar.update_frame(_make_frame(state="IDLE"))
        assert bar._lbl_elapsed.text() == "00:00.0"

    def test_elapsed_resets_on_ready(self, qtbot: QtBot) -> None:
        """Transitioning to READY resets the elapsed label."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING"))
        bar.update_frame(_make_frame(state="READY"))
        assert bar._lbl_elapsed.text() == "00:00.0"

    def test_elapsed_accumulates_while_running(self, qtbot: QtBot) -> None:
        """Elapsed counter starts when RUNNING begins."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING"))
        assert bar._running is True
        assert bar._elapsed_s == pytest.approx(0.0)

    def test_elapsed_freezes_on_pause(self, qtbot: QtBot) -> None:
        """Transitioning to PAUSED freezes elapsed without resetting it."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING"))
        # Simulate some time passing
        bar._run_start -= 2.0   # pretend 2 s have elapsed
        bar.update_frame(_make_frame(state="PAUSED"))
        assert bar._running is False
        assert bar._elapsed_s >= 2.0

    def test_elapsed_resumes_after_pause(self, qtbot: QtBot) -> None:
        """Returning to RUNNING after PAUSED continues accumulating from where it stopped."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        bar.update_frame(_make_frame(state="RUNNING"))
        bar._run_start -= 3.0
        bar.update_frame(_make_frame(state="PAUSED"))
        frozen = bar._elapsed_s
        bar.update_frame(_make_frame(state="RUNNING"))
        assert bar._running is True
        # Accumulated seconds must not have been reset
        assert bar._elapsed_s == pytest.approx(frozen)

    def test_all_valid_states_accepted(self, qtbot: QtBot) -> None:
        """update_frame() handles every valid Arduino state without raising."""
        bar = StatusBar()
        qtbot.addWidget(bar)
        for state in ("IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"):
            bar.update_frame(_make_frame(state=state))
            assert bar._lbl_state.text() == state
