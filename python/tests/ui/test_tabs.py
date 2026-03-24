"""
tests/ui/test_tabs.py
=====================

Unit tests for:
  * ui/tabs/control_tab.py    — ControlTab
  * ui/tabs/serial_monitor_tab.py — SerialMonitorTab

Run:
    uv run pytest tests/ui/test_tabs.py -v
"""

from __future__ import annotations

import queue
from unittest.mock import MagicMock, call, patch

import pytest
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication

from core.command_interface import CommandError
from core.profile import DipProfile
from ui.tabs.control_tab import ControlTab
from ui.tabs.serial_monitor_tab import SerialMonitorTab


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_profile(name: str = "test") -> DipProfile:
    return DipProfile(
        name=name,
        dip_speed_mm_s=5.0,
        withdraw_speed_mm_s=5.0,
        accel_mm_s2=20.0,
        dip_depth_mm=10.0,
        dwell_bottom_ms=200,
        dwell_top_ms=100,
        n_dips=1,
    )


def _make_ci(**kwargs) -> MagicMock:
    ci = MagicMock()
    for k, v in kwargs.items():
        setattr(ci, k, v)
    return ci


def _make_manager(lines: list[str] | None = None) -> MagicMock:
    """Return a mock with a pre-filled raw_queue."""
    mgr = MagicMock()
    mgr.raw_queue = queue.Queue()
    if lines:
        for ln in lines:
            mgr.raw_queue.put_nowait(ln)
    return mgr


# ===========================================================================
# TestControlTab
# ===========================================================================

class TestControlTab:
    # ------------------------------------------------------------------
    # Initial state
    # ------------------------------------------------------------------

    def test_initial_buttons_disabled_without_ci(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        assert not tab._btn_home.isEnabled()
        assert not tab._btn_stop.isEnabled()
        assert not tab._btn_jog_up.isEnabled()
        assert not tab._btn_jog_down.isEnabled()
        assert not tab._btn_run.isEnabled()
        assert not tab._btn_pause.isEnabled()
        assert not tab._btn_resume.isEnabled()

    def test_connect_button_shows_connect_initially(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        assert tab._btn_connect.text() == "Connect"

    def test_port_and_baud_enabled_when_disconnected(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        assert tab._edit_port.isEnabled()
        assert tab._spin_baud.isEnabled()

    # ------------------------------------------------------------------
    # set_command_interface
    # ------------------------------------------------------------------

    def test_connect_button_shows_disconnect_when_ci_set(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        assert tab._btn_connect.text() == "Disconnect"

    def test_port_baud_disabled_when_connected(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        assert not tab._edit_port.isEnabled()
        assert not tab._spin_baud.isEnabled()

    def test_set_ci_none_restores_connect_button(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.set_command_interface(None)
        assert tab._btn_connect.text() == "Connect"

    # ------------------------------------------------------------------
    # update_state — button enable rules
    # ------------------------------------------------------------------

    @pytest.mark.parametrize("state,expected_home", [
        ("IDLE",    True),
        ("READY",   True),
        ("ERROR",   True),
        ("RUNNING", False),
        ("PAUSED",  False),
        ("HOMING",  False),
    ])
    def test_home_enabled_only_in_correct_states(self, qtbot, state, expected_home):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.update_state(state)
        assert tab._btn_home.isEnabled() == expected_home

    @pytest.mark.parametrize("state,expected_stop", [
        ("RUNNING", True),
        ("PAUSED",  True),
        ("IDLE",    False),
        ("READY",   False),
        ("ERROR",   False),
    ])
    def test_stop_enabled_only_in_correct_states(self, qtbot, state, expected_stop):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.update_state(state)
        assert tab._btn_stop.isEnabled() == expected_stop

    def test_jog_enabled_only_in_ready(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        for state in ("IDLE", "HOMING", "RUNNING", "PAUSED", "ERROR"):
            tab.update_state(state)
            assert not tab._btn_jog_up.isEnabled(), f"jog_up should be disabled in {state}"
            assert not tab._btn_jog_down.isEnabled()
        tab.update_state("READY")
        assert tab._btn_jog_up.isEnabled()
        assert tab._btn_jog_down.isEnabled()

    def test_run_enabled_only_when_ready_and_profiles_loaded(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.set_profiles([_make_profile()])
        tab.update_state("READY")
        assert tab._btn_run.isEnabled()
        tab.update_state("IDLE")
        assert not tab._btn_run.isEnabled()

    def test_run_disabled_without_profiles_even_when_ready(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.update_state("READY")
        assert not tab._btn_run.isEnabled()

    def test_pause_enabled_only_in_running(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.update_state("RUNNING")
        assert tab._btn_pause.isEnabled()
        tab.update_state("READY")
        assert not tab._btn_pause.isEnabled()

    def test_resume_enabled_only_in_paused(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.update_state("PAUSED")
        assert tab._btn_resume.isEnabled()
        tab.update_state("RUNNING")
        assert not tab._btn_resume.isEnabled()

    # ------------------------------------------------------------------
    # set_profiles
    # ------------------------------------------------------------------

    def test_set_profiles_populates_combo(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_profiles([_make_profile("A"), _make_profile("B")])
        assert tab._combo_profile.count() == 2
        assert tab._combo_profile.itemText(0) == "A"
        assert tab._combo_profile.itemText(1) == "B"

    def test_set_profiles_preserves_selection(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_profiles([_make_profile("A"), _make_profile("B")])
        tab._combo_profile.setCurrentIndex(1)
        tab.set_profiles([_make_profile("A"), _make_profile("B"), _make_profile("C")])
        assert tab._combo_profile.currentText() == "B"

    # ------------------------------------------------------------------
    # Signals
    # ------------------------------------------------------------------

    def test_connect_signal_emitted_with_port_and_baud(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab._edit_port.setText("COM7")
        tab._spin_baud.setValue(9600)
        with qtbot.waitSignal(tab.connect_requested, timeout=500) as blocker:
            tab._btn_connect.click()
        assert blocker.args == ["COM7", 9600]

    def test_disconnect_signal_emitted_when_connected(self, qtbot):
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        with qtbot.waitSignal(tab.disconnect_requested, timeout=500):
            tab._btn_connect.click()

    # ------------------------------------------------------------------
    # Button click → CommandInterface calls
    # ------------------------------------------------------------------

    def test_home_calls_ci_home(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("READY")
        tab._btn_home.click()
        ci.home.assert_called_once()

    def test_stop_calls_ci_stop(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("RUNNING")
        tab._btn_stop.click()
        ci.stop.assert_called_once()

    def test_jog_up_calls_ci_jog(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("READY")
        tab._spin_jog_spd.setValue(3.5)
        tab._btn_jog_up.click()
        ci.jog.assert_called_once_with("UP", 3.5)

    def test_jog_down_calls_ci_jog(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("READY")
        tab._spin_jog_spd.setValue(2.0)
        tab._btn_jog_down.click()
        ci.jog.assert_called_once_with("DOWN", 2.0)

    def test_pause_calls_ci_pause(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("RUNNING")
        tab._btn_pause.click()
        ci.pause.assert_called_once()

    def test_resume_calls_ci_resume(self, qtbot):
        ci = _make_ci()
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("PAUSED")
        tab._btn_resume.click()
        ci.resume.assert_called_once()

    def test_command_error_does_not_propagate(self, qtbot):
        """A CommandError from the CI must not raise — it should show a dialog."""
        ci = _make_ci()
        ci.home.side_effect = CommandError("test error")
        tab = ControlTab()
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab.update_state("READY")
        with patch("ui.tabs.control_tab.QMessageBox"):
            tab._btn_home.click()   # must not raise


# ===========================================================================
# TestSerialMonitorTab
# ===========================================================================

class TestSerialMonitorTab:
    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def test_creates_without_error(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        assert tab._manager is None

    def test_log_pane_is_read_only(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        assert tab._log.isReadOnly()

    def test_log_max_block_count_is_bounded(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        assert tab._log.maximumBlockCount() == 2000

    # ------------------------------------------------------------------
    # set_manager
    # ------------------------------------------------------------------

    def test_set_manager_stores_reference(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        assert tab._manager is mgr

    def test_set_manager_none_accepted(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab.set_manager(None)
        assert tab._manager is None

    # ------------------------------------------------------------------
    # Polling raw_queue
    # ------------------------------------------------------------------

    def test_poll_drains_raw_queue(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager(["TX CMD HOME", "RX ACK HOME"])
        tab.set_manager(mgr)
        tab._poll_raw_queue()
        assert mgr.raw_queue.empty()

    def test_poll_appends_to_log(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager(["TX CMD HOME"])
        tab.set_manager(mgr)
        tab._poll_raw_queue()
        assert "TX CMD HOME" in tab._log.toPlainText()

    def test_poll_does_nothing_without_manager(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        # Should not raise
        tab._poll_raw_queue()

    def test_poll_does_nothing_without_raw_queue(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = MagicMock(spec=[])   # no raw_queue attribute
        tab.set_manager(mgr)
        tab._poll_raw_queue()  # must not raise

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    def test_send_calls_manager_send_command(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()
        mgr.send_command.assert_called_once_with("CMD HOME")

    def test_send_clears_input(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()
        assert tab._input.toPlainText() == ""

    def test_send_multiple_lines_calls_send_for_each(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME\nCMD STATUS")
        tab._on_send()
        assert mgr.send_command.call_count == 2
        mgr.send_command.assert_any_call("CMD HOME")
        mgr.send_command.assert_any_call("CMD STATUS")

    def test_send_skips_blank_lines(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("\nCMD PING\n\n")
        tab._on_send()
        mgr.send_command.assert_called_once_with("CMD PING")

    def test_send_adds_to_history_sidebar(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()
        assert tab._history_list.count() == 1

    def test_send_does_nothing_without_manager(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()   # must not raise

    def test_send_error_appends_to_log(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        mgr.send_command.side_effect = RuntimeError("port closed")
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()
        assert "ERR" in tab._log.toPlainText()

    # ------------------------------------------------------------------
    # Quick send
    # ------------------------------------------------------------------

    def test_quick_send_calls_manager(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._combo_quick.setCurrentIndex(0)
        tab._on_quick_send()
        mgr.send_command.assert_called_once()

    def test_quick_send_does_nothing_without_manager(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._on_quick_send()   # must not raise

    # ------------------------------------------------------------------
    # History sidebar — click to reload
    # ------------------------------------------------------------------

    def test_history_click_reloads_into_input(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        mgr = _make_manager()
        tab.set_manager(mgr)
        tab._input.setPlainText("CMD HOME")
        tab._on_send()
        # Now click the history item
        item = tab._history_list.item(0)
        tab._on_history_clicked(item)
        assert "CMD HOME" in tab._input.toPlainText()

    # ------------------------------------------------------------------
    # Auto-scroll toggle
    # ------------------------------------------------------------------

    def test_autoscroll_on_by_default(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        assert tab._auto_scroll is True
        assert tab._chk_autoscroll.isChecked()

    def test_autoscroll_toggle_updates_flag(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._chk_autoscroll.setChecked(False)
        assert tab._auto_scroll is False

    # ------------------------------------------------------------------
    # _CommandInput — history navigation
    # ------------------------------------------------------------------

    def test_command_input_push_history_stores_batch(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._input.push_history("CMD HOME")
        assert tab._input._history == ["CMD HOME"]

    def test_command_input_push_history_most_recent_first(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._input.push_history("CMD HOME")
        tab._input.push_history("CMD STOP")
        assert tab._input._history[0] == "CMD STOP"
        assert tab._input._history[1] == "CMD HOME"

    def test_command_input_blank_batch_not_stored(self, qtbot):
        tab = SerialMonitorTab()
        qtbot.addWidget(tab)
        tab._input.push_history("   ")
        assert tab._input._history == []
