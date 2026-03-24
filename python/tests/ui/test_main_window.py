"""
tests/ui/test_main_window.py
============================

Unit tests for ui/main_window.py — MainWindow.

All tests use MockArduino; no hardware required.

Run:
    uv run pytest tests/ui/test_main_window.py -v
"""

from __future__ import annotations

import queue
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from core.profile import DipProfile, save_profile
from core.telemetry_parser import TelemetryFrame
from tests.mock_arduino import MockArduino
from ui.main_window import MainWindow


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_frame(
    state: str  = "READY",
    phase: str  = "NONE",
    pos:   float = 0.0,
    ts:    int   = 1000,
) -> TelemetryFrame:
    return TelemetryFrame(
        timestamp_ms       = ts,
        pos_mm             = pos,
        vel_actual_mm_s    = 0.0,
        vel_commanded_mm_s = 0.0,
        accel_mm_s2        = 0.0,
        state              = state,
        phase              = phase,
    )


def _make_profile(name: str = "test") -> DipProfile:
    return DipProfile(
        name                = name,
        dip_speed_mm_s      = 5.0,
        withdraw_speed_mm_s = 5.0,
        accel_mm_s2         = 20.0,
        dip_depth_mm        = 10.0,
        dwell_bottom_ms     = 200,
        dwell_top_ms        = 100,
        n_dips              = 1,
    )


def _connect_mock(win: MainWindow) -> MockArduino:
    """
    Simulate a connect by injecting a running MockArduino directly,
    bypassing the real SerialManager path.
    """
    mock = MockArduino(speed_multiplier=10.0, telem_hz_default=10)
    mock.start()
    from core.command_interface import CommandInterface
    win._manager = mock          # type: ignore[assignment]
    win._ci      = CommandInterface(mock)  # type: ignore[arg-type]
    win._broadcast_ci(win._ci)
    win._status.set_connected(True, "MOCK")
    return mock


# ===========================================================================
# TestMainWindow
# ===========================================================================

class TestMainWindow:

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def test_creates_without_error(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)

    def test_window_title_set(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert "Dip Coater" in win.windowTitle()

    def test_five_tabs_present(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert win._tabs.count() == 5

    def test_tab_labels(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        labels = [win._tabs.tabText(i) for i in range(win._tabs.count())]
        assert "Control"        in labels
        assert "Serial Monitor" in labels
        assert "Telemetry"      in labels
        assert "Profiles"       in labels
        assert "Profile Editor" in labels

    def test_estop_present(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert win._estop is not None

    def test_status_bar_present(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert win._status is not None

    def test_profiles_dir_created_on_init(self, qtbot, tmp_path):
        pdir = tmp_path / "myprofiles"
        win  = MainWindow(profile_dir=pdir, log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert pdir.exists()

    # ------------------------------------------------------------------
    # Initial disconnected state
    # ------------------------------------------------------------------

    def test_no_manager_on_init(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert win._manager is None
        assert win._ci      is None

    def test_control_tab_disabled_before_connect(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert not win._tab_control._btn_home.isEnabled()

    # ------------------------------------------------------------------
    # _on_connect / _on_disconnect via MockArduino injection
    # ------------------------------------------------------------------

    def test_connect_sets_manager_and_ci(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            assert win._manager is mock
            assert win._ci      is not None
        finally:
            mock.stop()

    def test_connect_enables_control_tab(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            assert win._tab_control._btn_home.isEnabled()
        finally:
            mock.stop()

    def test_connect_enables_telem_set_rate(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            assert win._tab_telem._btn_set_rate.isEnabled()
        finally:
            mock.stop()

    def test_disconnect_clears_manager_and_ci(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        win._on_disconnect()
        assert win._manager is None
        assert win._ci      is None
        mock.stop()

    def test_disconnect_disables_control_tab(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        win._on_disconnect()
        assert not win._tab_control._btn_home.isEnabled()
        mock.stop()

    def test_disconnect_disables_telem_set_rate(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        win._on_disconnect()
        assert not win._tab_telem._btn_set_rate.isEnabled()
        mock.stop()

    # ------------------------------------------------------------------
    # Telemetry fan-out
    # ------------------------------------------------------------------

    def test_poll_telem_updates_control_tab_state(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            mock.telem_queue.put_nowait(_make_frame(state="RUNNING"))
            win._poll_telem()
            assert win._tab_control._state == "RUNNING"
        finally:
            mock.stop()

    def test_poll_telem_updates_telemetry_tab(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            mock.telem_queue.put_nowait(_make_frame(pos=42.5))
            win._poll_telem()
            assert "42.50" in win._tab_telem._lbl_pos.text()
        finally:
            mock.stop()

    def test_poll_telem_drains_multiple_frames(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        try:
            for i in range(5):
                mock.telem_queue.put_nowait(_make_frame(ts=1000 + i * 100))
            win._poll_telem()
            assert mock.telem_queue.empty()
        finally:
            mock.stop()

    def test_poll_telem_does_nothing_when_disconnected(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        win._poll_telem()   # must not raise

    # ------------------------------------------------------------------
    # Profile selection fan-out
    # ------------------------------------------------------------------

    def test_profile_selected_populates_control_tab(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        p = _make_profile("myrun")
        win._on_profile_selected(p)
        assert win._tab_control._combo_profile.count() == 1
        assert win._tab_control._combo_profile.itemText(0) == "myrun"

    def test_profile_selected_updates_telem_profile_name(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        win._on_profile_selected(_make_profile("coating_v2"))
        assert win._tab_telem._profile_name == "coating_v2"

    def test_profile_selected_switches_to_control_tab(self, qtbot, tmp_path):
        win = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        win._tabs.setCurrentIndex(3)   # start on Profiles tab
        win._on_profile_selected(_make_profile("x"))
        assert win._tabs.currentWidget() is win._tab_control

    # ------------------------------------------------------------------
    # Initial profile loading
    # ------------------------------------------------------------------

    def test_startup_loads_profiles_from_disk(self, qtbot, tmp_path):
        pdir = tmp_path / "profiles"
        pdir.mkdir()
        save_profile(_make_profile("alpha"), pdir / "alpha.json")
        save_profile(_make_profile("beta"),  pdir / "beta.json")
        win = MainWindow(profile_dir=pdir, log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        assert win._tab_control._combo_profile.count() == 2

    # ------------------------------------------------------------------
    # closeEvent
    # ------------------------------------------------------------------

    def test_close_event_stops_manager(self, qtbot, tmp_path):
        win  = MainWindow(profile_dir=tmp_path / "profiles", log_dir=tmp_path / "logs")
        qtbot.addWidget(win)
        mock = _connect_mock(win)
        win.close()
        assert win._manager is None
        mock.stop()   # safe to call again — stop() is idempotent
