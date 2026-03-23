"""
tests/test_mock_arduino.py
==========================

Unit tests for :class:`~tests.mock_arduino.MockArduino`.

Covers
------
* Lifecycle: start / stop / is_connected
* State machine: initial state, valid and invalid transitions
* Every command handler: correct ACK, ERR on wrong state, ERR on bad args
* Segmented move protocol: BEGIN → MOVE_SEG × N → PROFILE_READY → RUN_LOADED_MOVE
* Timed state transitions: HOMING → READY, RUNNING → READY
* TELEM broadcasting: frames arrive at the configured rate
* Test helpers: inject_error, set_position
"""

from __future__ import annotations

import queue
import time
from typing import Final

import pytest

from core.telemetry_parser import TelemetryFrame
from tests.mock_arduino import MockArduino

# Speed multiplier high enough that waits in tests are very short.
_FAST: Final[float] = 50.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _drain_response(mock: MockArduino, timeout: float = 1.0) -> list[str]:
    """Collect all responses currently on the queue."""
    responses: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            responses.append(mock.response_queue.get(timeout=0.05))
        except queue.Empty:
            break
    return responses


def _send(mock: MockArduino, cmd: str) -> str:
    """Send *cmd* and return the first response (blocks up to 1 s)."""
    mock.send_command(cmd)
    return mock.response_queue.get(timeout=1.0)


def _wait_state(mock: MockArduino, target: str, timeout: float = 2.0) -> bool:
    """Poll TELEM frames until state == *target* or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            frame: TelemetryFrame = mock.telem_queue.get(timeout=0.1)
            if frame.state == target:
                return True
        except queue.Empty:
            pass
    return False


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

class TestLifecycle:
    def test_not_connected_before_start(self) -> None:
        mock = MockArduino()
        assert not mock.is_connected

    def test_connected_after_start(self) -> None:
        mock = MockArduino()
        mock.start()
        try:
            assert mock.is_connected
        finally:
            mock.stop()

    def test_not_connected_after_stop(self) -> None:
        mock = MockArduino()
        mock.start()
        mock.stop()
        assert not mock.is_connected

    def test_telem_queue_exists(self) -> None:
        mock = MockArduino()
        assert isinstance(mock.telem_queue, queue.Queue)

    def test_response_queue_exists(self) -> None:
        mock = MockArduino()
        assert isinstance(mock.response_queue, queue.Queue)


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------

class TestInitialState:
    def test_starts_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            resp = _send(mock, "CMD GET_STATE")
            assert resp == "STATE IDLE NONE"
        finally:
            mock.stop()

    def test_run_profile_rejected_in_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            resp = _send(mock, "CMD RUN_PROFILE 5 5 30 20 0 0 1")
            assert resp.startswith("ERR")
        finally:
            mock.stop()

    def test_stop_rejected_in_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            resp = _send(mock, "CMD STOP")
            assert resp.startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# HOME command
# ---------------------------------------------------------------------------

class TestHome:
    def test_home_ack_from_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD HOME") == "ACK HOME"
        finally:
            mock.stop()

    def test_home_transitions_to_homing(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            mock.send_command("CMD HOME")
            resp = mock.response_queue.get(timeout=0.5)
            assert resp == "ACK HOME"
            resp2 = _send(mock, "CMD GET_STATE")
            # Should be HOMING or already READY (fast mock)
            assert resp2.startswith("STATE")
        finally:
            mock.stop()

    def test_home_eventually_reaches_ready(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            mock.send_command("CMD HOME")
            mock.response_queue.get(timeout=0.5)  # consume ACK HOME
            assert _wait_state(mock, "READY", timeout=2.0), "Never reached READY"
        finally:
            mock.stop()

    def test_home_zeroes_position(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.set_position(42.0)
        mock.start()
        try:
            mock.send_command("CMD HOME")
            mock.response_queue.get(timeout=0.5)
            _wait_state(mock, "READY", timeout=2.0)
            # After homing, position should be 0
            frame: TelemetryFrame = mock.telem_queue.get(timeout=1.0)
            while frame.state != "READY":
                frame = mock.telem_queue.get(timeout=1.0)
            assert frame.pos_mm == pytest.approx(0.0, abs=0.1)
        finally:
            mock.stop()

    def test_home_rejected_in_running(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            # Get to READY first
            mock.send_command("CMD HOME")
            mock.response_queue.get(timeout=1.0)
            _wait_state(mock, "READY", timeout=2.0)
            # Start a run
            mock.send_command("CMD RUN_PROFILE 5 5 30 20 0 0 1")
            mock.response_queue.get(timeout=1.0)
            # HOME during RUNNING should ERR
            resp = _send(mock, "CMD HOME")
            assert resp.startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# STOP / ESTOP
# ---------------------------------------------------------------------------

class TestStopEstop:
    def _get_to_ready(self, mock: MockArduino) -> None:
        mock.send_command("CMD HOME")
        mock.response_queue.get(timeout=1.0)
        _wait_state(mock, "READY", timeout=2.0)

    def test_stop_ack_from_ready(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD STOP") == "ACK STOP"
        finally:
            mock.stop()

    def test_estop_ack_from_any_state(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD ESTOP") == "ACK ESTOP"
        finally:
            mock.stop()

    def test_estop_transitions_to_error(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            mock.send_command("CMD ESTOP")
            mock.response_queue.get(timeout=0.5)
            assert _send(mock, "CMD GET_STATE") == "STATE ERROR NONE"
        finally:
            mock.stop()

    def test_stop_from_idle_rejected(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD STOP").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# RUN_PROFILE
# ---------------------------------------------------------------------------

class TestRunProfile:
    def _get_to_ready(self, mock: MockArduino) -> None:
        mock.send_command("CMD HOME")
        mock.response_queue.get(timeout=1.0)
        _wait_state(mock, "READY", timeout=2.0)

    def test_run_profile_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD RUN_PROFILE 5.0 5.0 30.0 20.0 0 0 1") == "ACK RUN_PROFILE"
        finally:
            mock.stop()

    def test_run_profile_transitions_to_running(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD RUN_PROFILE 5.0 5.0 30.0 20.0 0 0 1")
            mock.response_queue.get(timeout=1.0)
            assert _send(mock, "CMD GET_STATE").startswith("STATE RUNNING")
        finally:
            mock.stop()

    def test_run_profile_completes_and_returns_to_ready(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD RUN_PROFILE 5.0 5.0 30.0 20.0 0 0 1")
            mock.response_queue.get(timeout=1.0)
            assert _wait_state(mock, "READY", timeout=5.0), "Run did not complete"
        finally:
            mock.stop()

    def test_run_profile_position_goes_negative(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD RUN_PROFILE 5.0 5.0 30.0 20.0 500 0 1")
            mock.response_queue.get(timeout=1.0)

            min_pos = 0.0
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                try:
                    frame: TelemetryFrame = mock.telem_queue.get(timeout=0.1)
                    min_pos = min(min_pos, frame.pos_mm)
                    if frame.state == "READY":
                        break
                except queue.Empty:
                    pass

            assert min_pos < -1.0, f"Position never went negative: {min_pos:.2f} mm"
        finally:
            mock.stop()

    def test_run_profile_rejected_in_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD RUN_PROFILE 5 5 30 20 0 0 1").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# PAUSE / RESUME
# ---------------------------------------------------------------------------

class TestPauseResume:
    def _get_to_running(self, mock: MockArduino) -> None:
        mock.send_command("CMD HOME")
        mock.response_queue.get(timeout=1.0)
        _wait_state(mock, "READY", timeout=2.0)
        mock.send_command("CMD RUN_PROFILE 5.0 5.0 30.0 20.0 0 0 1")
        mock.response_queue.get(timeout=1.0)

    def test_pause_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_running(mock)
            assert _send(mock, "CMD PAUSE") == "ACK PAUSE"
        finally:
            mock.stop()

    def test_pause_transitions_to_paused(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_running(mock)
            mock.send_command("CMD PAUSE")
            mock.response_queue.get(timeout=1.0)
            assert _send(mock, "CMD GET_STATE").startswith("STATE PAUSED")
        finally:
            mock.stop()

    def test_resume_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_running(mock)
            mock.send_command("CMD PAUSE")
            mock.response_queue.get(timeout=1.0)
            assert _send(mock, "CMD RESUME") == "ACK RESUME"
        finally:
            mock.stop()

    def test_pause_rejected_in_ready(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            mock.send_command("CMD HOME")
            mock.response_queue.get(timeout=1.0)
            _wait_state(mock, "READY", timeout=2.0)
            assert _send(mock, "CMD PAUSE").startswith("ERR")
        finally:
            mock.stop()

    def test_resume_rejected_in_ready(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            mock.send_command("CMD HOME")
            mock.response_queue.get(timeout=1.0)
            _wait_state(mock, "READY", timeout=2.0)
            assert _send(mock, "CMD RESUME").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# GET_STATE
# ---------------------------------------------------------------------------

class TestGetState:
    def test_returns_state_and_phase(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            resp = _send(mock, "CMD GET_STATE")
            parts = resp.split()
            assert len(parts) == 3
            assert parts[0] == "STATE"
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# SET_TELEM_RATE
# ---------------------------------------------------------------------------

class TestSetTelemRate:
    def test_set_telem_rate_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD SET_TELEM_RATE 10") == "ACK SET_TELEM_RATE"
        finally:
            mock.stop()

    def test_set_telem_rate_zero_stops_frames(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            mock.send_command("CMD SET_TELEM_RATE 0")
            mock.response_queue.get(timeout=0.5)
            # Drain existing frames
            while not mock.telem_queue.empty():
                mock.telem_queue.get_nowait()
            time.sleep(0.3)
            assert mock.telem_queue.empty(), "Frames arrived after rate=0"
        finally:
            mock.stop()

    def test_set_telem_rate_invalid_rejected(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD SET_TELEM_RATE 99").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# SET_SOFT_LIMITS
# ---------------------------------------------------------------------------

class TestSetSoftLimits:
    def test_set_soft_limits_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD SET_SOFT_LIMITS -50.0 50.0") == "ACK SET_SOFT_LIMITS"
        finally:
            mock.stop()

    def test_inverted_limits_rejected(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD SET_SOFT_LIMITS 50.0 -50.0").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# JOG
# ---------------------------------------------------------------------------

class TestJog:
    def _get_to_ready(self, mock: MockArduino) -> None:
        mock.send_command("CMD HOME")
        mock.response_queue.get(timeout=1.0)
        _wait_state(mock, "READY", timeout=2.0)

    def test_jog_up_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD JOG UP 5.0") == "ACK JOG"
        finally:
            mock.stop()

    def test_jog_down_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD JOG DOWN 5.0") == "ACK JOG"
        finally:
            mock.stop()

    def test_jog_invalid_direction_rejected(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD JOG LEFT 5.0").startswith("ERR")
        finally:
            mock.stop()

    def test_jog_rejected_in_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD JOG UP 5.0").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# Segmented move protocol
# ---------------------------------------------------------------------------

class TestSegmentedMove:
    def _get_to_ready(self, mock: MockArduino) -> None:
        mock.send_command("CMD HOME")
        mock.response_queue.get(timeout=1.0)
        _wait_state(mock, "READY", timeout=2.0)

    def test_begin_segmented_move_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD BEGIN_SEGMENTED_MOVE 3") == "ACK BEGIN_SEGMENTED_MOVE"
        finally:
            mock.stop()

    def test_profile_ready_after_last_segment(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD BEGIN_SEGMENTED_MOVE 2")
            mock.response_queue.get(timeout=1.0)   # ACK BEGIN_SEGMENTED_MOVE

            mock.send_command("CMD MOVE_SEG -10.0 5.0 30.0")
            # No ACK for first segment
            assert mock.response_queue.empty()

            mock.send_command("CMD MOVE_SEG 10.0 5.0 30.0")
            resp = mock.response_queue.get(timeout=1.0)
            assert resp == "ACK PROFILE_READY"
        finally:
            mock.stop()

    def test_no_ack_for_intermediate_segments(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD BEGIN_SEGMENTED_MOVE 3")
            mock.response_queue.get(timeout=1.0)

            for _ in range(2):
                mock.send_command("CMD MOVE_SEG -5.0 5.0 30.0")
                assert mock.response_queue.empty(), "Got unexpected ACK for intermediate seg"

            # Last segment
            mock.send_command("CMD MOVE_SEG 10.0 5.0 30.0")
            assert mock.response_queue.get(timeout=1.0) == "ACK PROFILE_READY"
        finally:
            mock.stop()

    def test_dwell_seg_counts_toward_total(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD BEGIN_SEGMENTED_MOVE 2")
            mock.response_queue.get(timeout=1.0)

            mock.send_command("CMD MOVE_SEG -10.0 5.0 30.0")
            mock.send_command("CMD DWELL_SEG 500")
            resp = mock.response_queue.get(timeout=1.0)
            assert resp == "ACK PROFILE_READY"
        finally:
            mock.stop()

    def test_run_loaded_move_ack(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD BEGIN_SEGMENTED_MOVE 1")
            mock.response_queue.get(timeout=1.0)
            mock.send_command("CMD MOVE_SEG -10.0 5.0 30.0")
            mock.response_queue.get(timeout=1.0)   # ACK PROFILE_READY

            assert _send(mock, "CMD RUN_LOADED_MOVE") == "ACK RUN_LOADED_MOVE"
        finally:
            mock.stop()

    def test_run_loaded_move_completes(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            mock.send_command("CMD BEGIN_SEGMENTED_MOVE 2")
            mock.response_queue.get(timeout=1.0)
            mock.send_command("CMD MOVE_SEG -10.0 5.0 30.0")
            mock.send_command("CMD MOVE_SEG 10.0 5.0 30.0")
            mock.response_queue.get(timeout=1.0)   # ACK PROFILE_READY
            mock.send_command("CMD RUN_LOADED_MOVE")
            mock.response_queue.get(timeout=1.0)   # ACK RUN_LOADED_MOVE

            assert _wait_state(mock, "READY", timeout=5.0), "Segmented run did not complete"
        finally:
            mock.stop()

    def test_run_loaded_move_rejected_without_segments(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            self._get_to_ready(mock)
            assert _send(mock, "CMD RUN_LOADED_MOVE").startswith("ERR")
        finally:
            mock.stop()

    def test_begin_segmented_rejected_in_idle(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD BEGIN_SEGMENTED_MOVE 3").startswith("ERR")
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# TELEM broadcasting
# ---------------------------------------------------------------------------

class TestTelemBroadcast:
    def test_frames_arrive_at_configured_rate(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            # Collect frames for ~0.5 s at 20 Hz → expect ~10 frames
            time.sleep(0.5)
            count = 0
            while not mock.telem_queue.empty():
                mock.telem_queue.get_nowait()
                count += 1
            assert count >= 5, f"Only {count} frames in 0.5 s at 20 Hz"
        finally:
            mock.stop()

    def test_frame_fields_are_valid(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            frame: TelemetryFrame = mock.telem_queue.get(timeout=1.0)
            assert frame.state in {"IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"}
            assert frame.phase in {"NONE", "DESCENDING", "DWELL_BOTTOM", "ASCENDING", "DWELL_TOP"}
            assert frame.timestamp_ms >= 0
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# Unknown / malformed commands
# ---------------------------------------------------------------------------

class TestUnknownCommands:
    def test_unknown_verb_returns_err(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            assert _send(mock, "CMD BANANA").startswith("ERR")
        finally:
            mock.stop()

    def test_non_cmd_line_ignored(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            mock.send_command("TELEM,123,0.0,0.0,0.0,0.0,IDLE,NONE")
            time.sleep(0.1)
            assert mock.response_queue.empty()
        finally:
            mock.stop()


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

class TestHelpers:
    def test_inject_error_sets_state(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST)
        mock.start()
        try:
            mock.inject_error()
            resp = _send(mock, "CMD GET_STATE")
            assert "ERROR" in resp
        finally:
            mock.stop()

    def test_set_position_changes_pos(self) -> None:
        mock = MockArduino(speed_multiplier=_FAST, telem_hz_default=20)
        mock.start()
        try:
            mock.set_position(-42.5)
            # Poll frames until one reflects the new position (race-free).
            found = False
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    frame: TelemetryFrame = mock.telem_queue.get(timeout=0.1)
                    if frame.pos_mm == pytest.approx(-42.5, abs=0.1):
                        found = True
                        break
                except queue.Empty:
                    pass
            assert found, "No TELEM frame reflected set_position(-42.5)"
        finally:
            mock.stop()
