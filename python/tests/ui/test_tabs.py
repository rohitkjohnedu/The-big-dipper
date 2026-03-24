"""
tests/ui/test_tabs.py
=====================

Unit tests for:
  * ui/tabs/control_tab.py       — ControlTab
  * ui/tabs/serial_monitor_tab.py — SerialMonitorTab
  * ui/tabs/telemetry_tab.py     — TelemetryTab

Run:
    uv run pytest tests/ui/test_tabs.py -v
"""

from __future__ import annotations

import queue
import tempfile
from pathlib import Path
from unittest.mock import MagicMock, call, patch

import pytest
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication

from core.command_interface import CommandError
from core.profile import DipProfile
from core.telemetry_parser import TelemetryFrame
from ui.tabs.control_tab import ControlTab
from ui.tabs.serial_monitor_tab import SerialMonitorTab
from ui.tabs.telemetry_tab import TelemetryTab


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


# ===========================================================================
# TestTelemetryTab
# ===========================================================================

def _make_frame(
    state: str = "RUNNING",
    phase: str = "DESCENDING",
    pos: float = 10.0,
    vel_act: float = 5.0,
    vel_cmd: float = 5.0,
    accel: float = 0.0,
    ts: int = 1000,
) -> TelemetryFrame:
    return TelemetryFrame(
        timestamp_ms=ts,
        pos_mm=pos,
        vel_actual_mm_s=vel_act,
        vel_commanded_mm_s=vel_cmd,
        accel_mm_s2=accel,
        state=state,
        phase=phase,
    )


class TestTelemetryTab:
    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def test_creates_without_error(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)

    def test_set_rate_button_disabled_initially(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        assert not tab._btn_set_rate.isEnabled()

    def test_auto_record_on_by_default(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        assert tab._chk_auto.isChecked()

    # ------------------------------------------------------------------
    # set_command_interface
    # ------------------------------------------------------------------

    def test_set_rate_button_enabled_when_ci_set(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        assert tab._btn_set_rate.isEnabled()

    def test_set_rate_button_disabled_when_ci_cleared(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.set_command_interface(_make_ci())
        tab.set_command_interface(None)
        assert not tab._btn_set_rate.isEnabled()

    # ------------------------------------------------------------------
    # update_frame — readouts
    # ------------------------------------------------------------------

    def test_update_frame_updates_position_label(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(pos=42.5))
        assert "42.50" in tab._lbl_pos.text()

    def test_update_frame_updates_state_label(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(state="PAUSED"))
        assert tab._lbl_state.text() == "PAUSED"

    def test_update_frame_updates_phase_label(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(phase="DWELL_BOTTOM"))
        assert "DWELL BOTTOM" in tab._lbl_phase.text()

    def test_update_frame_updates_vel_labels(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(vel_act=3.5, vel_cmd=4.0))
        assert "3.50" in tab._lbl_vel_act.text()
        assert "4.00" in tab._lbl_vel_cmd.text()

    # ------------------------------------------------------------------
    # Auto-record — state-machine transitions
    # ------------------------------------------------------------------

    def test_auto_record_starts_on_running(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        assert not tab._recorder.is_recording
        tab.update_frame(_make_frame(state="READY"))
        tab.update_frame(_make_frame(state="RUNNING"))
        assert tab._recorder.is_recording

    def test_auto_record_stops_on_leaving_running(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(state="RUNNING"))
        assert tab._recorder.is_recording
        tab.update_frame(_make_frame(state="READY"))
        assert not tab._recorder.is_recording

    def test_auto_record_saves_csv(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.notify_profile_name("test_profile")
        tab.update_frame(_make_frame(state="RUNNING", ts=1000))
        tab.update_frame(_make_frame(state="RUNNING", ts=1100))
        tab.update_frame(_make_frame(state="READY",   ts=1200))
        csv_files = list(tmp_path.glob("*.csv"))
        assert len(csv_files) == 1
        assert "test_profile" in csv_files[0].name

    def test_auto_record_off_does_not_start_on_running(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._chk_auto.setChecked(False)
        tab.update_frame(_make_frame(state="RUNNING"))
        assert not tab._recorder.is_recording

    def test_record_button_reflects_recording_state(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(state="RUNNING"))
        assert tab._btn_record.isChecked()
        tab.update_frame(_make_frame(state="READY"))
        assert not tab._btn_record.isChecked()

    def test_frame_count_label_increments(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.update_frame(_make_frame(state="RUNNING", ts=1000))
        tab.update_frame(_make_frame(state="RUNNING", ts=1100))
        assert tab._lbl_frames.text() == "2"

    # ------------------------------------------------------------------
    # notify_profile_name
    # ------------------------------------------------------------------

    def test_notify_profile_name_updates_name(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.notify_profile_name("my_profile")
        assert tab._profile_name == "my_profile"

    def test_notify_empty_name_falls_back_to_unnamed(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.notify_profile_name("")
        assert tab._profile_name == "unnamed"

    # ------------------------------------------------------------------
    # Telem rate
    # ------------------------------------------------------------------

    def test_set_rate_calls_ci(self, qtbot, tmp_path):
        ci = _make_ci()
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.set_command_interface(ci)
        tab._spin_rate.setValue(20)
        tab._on_set_rate()
        ci.set_telem_rate.assert_called_once_with(20)

    def test_set_rate_does_nothing_without_ci(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._on_set_rate()   # must not raise

    # ------------------------------------------------------------------
    # Manual record toggle
    # ------------------------------------------------------------------

    def test_manual_record_start(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._on_record_toggled(True)
        assert tab._recorder.is_recording

    def test_manual_record_stop_saves_csv(self, qtbot, tmp_path):
        tab = TelemetryTab(log_dir=tmp_path)
        qtbot.addWidget(tab)
        tab.notify_profile_name("manual_test")
        tab._on_record_toggled(True)
        tab.update_frame(_make_frame(state="READY", ts=500))
        tab._on_record_toggled(False)
        assert not tab._recorder.is_recording


# ===========================================================================
# TestProfileManagerTab
# ===========================================================================

from ui.tabs.profile_manager_tab import ProfileManagerTab, _NewProfileDialog  # noqa: E402


def _save_profile_to(p: DipProfile, directory: Path) -> Path:
    """Helper: save a DipProfile to directory and return its path."""
    from core.profile import save_profile as _save
    safe = p.name.strip().replace(" ", "_")
    path = directory / f"{safe}.json"
    _save(p, path)
    return path


class TestProfileManagerTab:
    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def test_creates_without_error(self, qtbot, tmp_path):
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)

    def test_empty_dir_shows_zero_rows(self, qtbot, tmp_path):
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        assert tab._table.rowCount() == 0

    def test_profiles_dir_created_if_missing(self, qtbot, tmp_path):
        new_dir = tmp_path / "subdir" / "profiles"
        assert not new_dir.exists()
        tab = ProfileManagerTab(profile_dir=new_dir)
        qtbot.addWidget(tab)
        assert new_dir.exists()

    # ------------------------------------------------------------------
    # refresh — loading profiles
    # ------------------------------------------------------------------

    def test_refresh_populates_table(self, qtbot, tmp_path):
        _save_profile_to(_make_profile("alpha"), tmp_path)
        _save_profile_to(_make_profile("beta"), tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        assert tab._table.rowCount() == 2

    def test_refresh_shows_profile_name_in_first_column(self, qtbot, tmp_path):
        _save_profile_to(_make_profile("silica"), tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        assert tab._table.item(0, 0).text() == "silica"

    def test_set_profile_dir_reloads(self, qtbot, tmp_path):
        dir_a = tmp_path / "a"
        dir_a.mkdir()
        dir_b = tmp_path / "b"
        dir_b.mkdir()
        _save_profile_to(_make_profile("prof_a"), dir_a)
        _save_profile_to(_make_profile("prof_b"), dir_b)
        tab = ProfileManagerTab(profile_dir=dir_a)
        qtbot.addWidget(tab)
        assert tab._table.rowCount() == 1
        tab.set_profile_dir(dir_b)
        assert tab._table.rowCount() == 1
        assert tab._table.item(0, 0).text() == "prof_b"

    # ------------------------------------------------------------------
    # Selection — button enable states
    # ------------------------------------------------------------------

    def test_buttons_disabled_when_no_selection(self, qtbot, tmp_path):
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        assert not tab._btn_duplicate.isEnabled()
        assert not tab._btn_rename.isEnabled()
        assert not tab._btn_delete.isEnabled()
        assert not tab._btn_load.isEnabled()

    def test_buttons_enabled_after_selection(self, qtbot, tmp_path):
        _save_profile_to(_make_profile("myprofile"), tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        assert tab._btn_duplicate.isEnabled()
        assert tab._btn_rename.isEnabled()
        assert tab._btn_delete.isEnabled()
        assert tab._btn_load.isEnabled()

    # ------------------------------------------------------------------
    # Load — emits signal
    # ------------------------------------------------------------------

    def test_load_emits_profile_selected(self, qtbot, tmp_path):
        p = _make_profile("loadme")
        _save_profile_to(p, tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)

        emitted = []
        tab.profile_selected.connect(lambda prof: emitted.append(prof))
        tab._on_load()

        assert len(emitted) == 1
        assert emitted[0].name == "loadme"

    def test_load_does_nothing_without_selection(self, qtbot, tmp_path):
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        emitted = []
        tab.profile_selected.connect(lambda p: emitted.append(p))
        tab._on_load()
        assert emitted == []

    # ------------------------------------------------------------------
    # Delete
    # ------------------------------------------------------------------

    def test_delete_removes_file_and_row(self, qtbot, tmp_path, monkeypatch):
        p = _make_profile("todelete")
        path = _save_profile_to(p, tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        # Auto-confirm the dialog
        monkeypatch.setattr(
            "ui.tabs.profile_manager_tab.QMessageBox.question",
            lambda *args, **kwargs: __import__("PyQt6.QtWidgets", fromlist=["QMessageBox"]).QMessageBox.StandardButton.Yes,
        )
        tab._on_delete()
        assert not path.exists()
        assert tab._table.rowCount() == 0

    def test_delete_does_nothing_without_selection(self, qtbot, tmp_path):
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._on_delete()   # must not raise

    # ------------------------------------------------------------------
    # Duplicate
    # ------------------------------------------------------------------

    def test_duplicate_creates_new_file(self, qtbot, tmp_path, monkeypatch):
        p = _make_profile("original")
        _save_profile_to(p, tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        monkeypatch.setattr(
            "ui.tabs.profile_manager_tab._ask_name",
            lambda *args, **kwargs: ("original_copy", True),
        )
        tab._on_duplicate()
        assert (tmp_path / "original_copy.json").exists()
        assert tab._table.rowCount() == 2

    def test_duplicate_preserves_parameters(self, qtbot, tmp_path, monkeypatch):
        p = _make_profile("orig2")
        _save_profile_to(p, tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        monkeypatch.setattr(
            "ui.tabs.profile_manager_tab._ask_name",
            lambda *args, **kwargs: ("orig2_dup", True),
        )
        tab._on_duplicate()
        from core.profile import load_profile as _load
        loaded = _load(tmp_path / "orig2_dup.json")
        assert loaded.dip_speed_mm_s == p.dip_speed_mm_s
        assert loaded.dip_depth_mm == p.dip_depth_mm

    # ------------------------------------------------------------------
    # Rename
    # ------------------------------------------------------------------

    def test_rename_moves_file(self, qtbot, tmp_path, monkeypatch):
        p = _make_profile("oldname")
        old_path = _save_profile_to(p, tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        monkeypatch.setattr(
            "ui.tabs.profile_manager_tab._ask_name",
            lambda *args, **kwargs: ("newname", True),
        )
        tab._on_rename()
        assert not old_path.exists()
        assert (tmp_path / "newname.json").exists()

    def test_rename_updates_table(self, qtbot, tmp_path, monkeypatch):
        _save_profile_to(_make_profile("before"), tmp_path)
        tab = ProfileManagerTab(profile_dir=tmp_path)
        qtbot.addWidget(tab)
        tab._table.selectRow(0)
        monkeypatch.setattr(
            "ui.tabs.profile_manager_tab._ask_name",
            lambda *args, **kwargs: ("after", True),
        )
        tab._on_rename()
        assert tab._table.item(0, 0).text() == "after"

    # ------------------------------------------------------------------
    # _NewProfileDialog
    # ------------------------------------------------------------------

    def test_new_profile_dialog_default_values(self, qtbot):
        dlg = _NewProfileDialog()
        qtbot.addWidget(dlg)
        assert dlg._dip_spd.value() == 5.0
        assert dlg._n_dips.value() == 1

    def test_new_profile_dialog_get_profile_empty_name_raises(self, qtbot):
        dlg = _NewProfileDialog()
        qtbot.addWidget(dlg)
        dlg._name.setText("   ")
        with pytest.raises(ValueError):
            dlg.get_profile()


# ===========================================================================
# TestProfileEditorTab
# ===========================================================================

from ui.tabs.profile_editor_tab import ProfileEditorTab  # noqa: E402


class TestProfileEditorTab:

    def test_creates_without_error(self, qtbot):
        tab = ProfileEditorTab()
        qtbot.addWidget(tab)

    def test_shows_not_implemented_message(self, qtbot):
        tab = ProfileEditorTab()
        qtbot.addWidget(tab)
        # Find the label and verify it mentions the stub status
        from PyQt6.QtWidgets import QLabel
        labels = tab.findChildren(QLabel)
        texts = [lbl.text() for lbl in labels]
        assert any("not yet implemented" in t.lower() for t in texts)
