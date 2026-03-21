"""
tests/core/test_command_interface.py
=====================================

Tests for ``core/command_interface.py`` — ``CommandInterface`` and
:class:`~core.command_interface.CommandError`.

Covers:
- Correct command string construction for every public method.
- ACK verification: normal ACK, ERR response, timeout.
- ``estop()`` does not consume an ACK.
- ``get_state()`` parses the STATE response correctly.
- ``jog()`` direction validation and optional accel argument.
- ``run()`` dispatch: trapezoidal vs segmented vs spline vs unknown.
- ``_run_segmented()`` protocol order (BEGIN → segments → PROFILE_READY → RUN_LOADED_MOVE).
- Segment count validation: empty list, count exceeding MOVE_SEG_BUFFER_SIZE.
- Float formatting: 4 decimal places.

All tests run without hardware — no serial port is required.  A lightweight
``_FakeManager`` stands in for :class:`~core.serial_manager.SerialManager`.
"""

import queue
from typing import Any, Final

import pytest

from core.command_interface import (
    MOVE_SEG_BUFFER_SIZE,
    CommandError,
    CommandInterface,
)
from core.profile import DipProfile


# ---------------------------------------------------------------------------
# Fake SerialManager
# ---------------------------------------------------------------------------

class _FakeManager:
    """
    Minimal stand-in for :class:`~core.serial_manager.SerialManager`.

    Records every :meth:`send_command` call (without the trailing newline)
    in :attr:`sent` and exposes a real :class:`queue.Queue` for injecting
    mock Arduino responses.

    Attributes:
        sent:           List of command strings received, in call order.
                        Trailing newlines are stripped so tests can compare
                        against bare command strings.
        response_queue: Pre-loaded with ACK/ERR/STATE strings before each
                        test that needs a response.
    """

    def __init__(self) -> None:
        """Initialise with an empty command log and a fresh response queue."""
        self.sent:           list[str]      = []
        self.response_queue: queue.Queue[str] = queue.Queue()

    def send_command(self, cmd: str) -> None:
        """
        Record the command string (strip trailing newline for readability).

        Args:
            cmd: Command string as it would be sent to the Arduino.
        """
        self.sent.append(cmd.rstrip("\n"))


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_trapezoidal_profile(**kwargs: Any) -> DipProfile:
    """
    Build a valid trapezoidal ``DipProfile`` with sensible defaults.

    Args:
        **kwargs: Field overrides passed directly to ``DipProfile``.

    Returns:
        A fully validated trapezoidal ``DipProfile`` instance.
    """
    defaults: dict[str, Any] = dict(
        name                 = "test",
        dip_speed_mm_s       = 10.0,
        withdraw_speed_mm_s  = 15.0,
        accel_mm_s2          = 50.0,
        dip_depth_mm         = 100.0,
        dwell_bottom_ms      = 1000,
        dwell_top_ms         = 500,
        n_dips               = 3,
        velocity_profile_type = "trapezoidal",
    )
    defaults.update(kwargs)
    return DipProfile(**defaults)


def make_segmented_profile(
    segments: list[dict[str, Any]] | None = None,
) -> DipProfile:
    """
    Build a valid segmented ``DipProfile``.

    Args:
        segments: Optional list of segment dicts.  Defaults to a minimal
                  three-step descent/dwell/ascent sequence.

    Returns:
        A fully validated segmented ``DipProfile`` instance.
    """
    if segments is None:
        segments = [
            {"type": "move",  "distance_mm": -50.0, "speed_mm_s": 10.0, "accel_mm_s2": 50.0},
            {"type": "dwell", "duration_ms": 1000},
            {"type": "move",  "distance_mm":  50.0, "speed_mm_s": 15.0, "accel_mm_s2": 50.0},
        ]
    return DipProfile(
        name                  = "seg_test",
        dip_speed_mm_s        = 10.0,
        withdraw_speed_mm_s   = 15.0,
        accel_mm_s2           = 50.0,
        dip_depth_mm          = 100.0,
        dwell_bottom_ms       = 0,
        dwell_top_ms          = 0,
        n_dips                = 1,
        velocity_profile_type = "segmented",
        velocity_profile_data = {"segments": segments},
    )


def make_ci(
    fake: _FakeManager,
    ack_timeout_s: float = 1.0,
) -> CommandInterface:
    """
    Build a ``CommandInterface`` wired to a ``_FakeManager``.

    Args:
        fake:          The fake manager to inject.
        ack_timeout_s: Short timeout so tests fail fast.

    Returns:
        A :class:`~core.command_interface.CommandInterface` instance.
    """
    return CommandInterface(manager=fake, ack_timeout_s=ack_timeout_s)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Zero-argument commands
# ---------------------------------------------------------------------------

class TestHome:
    """CMD HOME command string and ACK verification."""

    def test_sends_correct_command(self) -> None:
        """home() must send exactly 'CMD HOME'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK HOME")
        make_ci(fake).home()
        assert fake.sent[0] == "CMD HOME"

    def test_raises_on_err_response(self) -> None:
        """home() must raise CommandError when the Arduino replies with ERR."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ERR HOME invalid_state")
        with pytest.raises(CommandError, match="invalid_state"):
            make_ci(fake).home()

    def test_raises_on_timeout(self) -> None:
        """home() must raise CommandError when no ACK arrives."""
        fake: _FakeManager = _FakeManager()
        with pytest.raises(CommandError, match="Timeout"):
            make_ci(fake, ack_timeout_s=0.05).home()


class TestStop:
    """CMD STOP command string."""

    def test_sends_correct_command(self) -> None:
        """stop() must send exactly 'CMD STOP'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK STOP")
        make_ci(fake).stop()
        assert fake.sent[0] == "CMD STOP"


class TestEstop:
    """CMD ESTOP bypasses ACK waiting."""

    def test_sends_correct_command(self) -> None:
        """estop() must send 'CMD ESTOP'."""
        fake: _FakeManager = _FakeManager()
        make_ci(fake).estop()
        assert fake.sent[0] == "CMD ESTOP"

    def test_does_not_consume_ack(self) -> None:
        """
        estop() must not wait for or consume an ACK.

        Pre-loading the queue with an unrelated ACK verifies that estop()
        returns immediately and leaves the queue item untouched.
        """
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK HOME")   # unrelated item
        make_ci(fake).estop()
        # The HOME ACK must still be in the queue.
        assert not fake.response_queue.empty()


class TestPause:
    """CMD PAUSE command string."""

    def test_sends_correct_command(self) -> None:
        """pause() must send exactly 'CMD PAUSE'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK PAUSE")
        make_ci(fake).pause()
        assert fake.sent[0] == "CMD PAUSE"


class TestResume:
    """CMD RESUME command string."""

    def test_sends_correct_command(self) -> None:
        """resume() must send exactly 'CMD RESUME'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK RESUME")
        make_ci(fake).resume()
        assert fake.sent[0] == "CMD RESUME"


# ---------------------------------------------------------------------------
# GET_STATE
# ---------------------------------------------------------------------------

class TestGetState:
    """CMD GET_STATE parses the STATE response."""

    def test_returns_state_phase_tuple(self) -> None:
        """get_state() must return (state, phase) parsed from 'STATE <s> <p>'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("STATE RUNNING DESCENDING")
        state: str
        phase: str
        state, phase = make_ci(fake).get_state()
        assert state == "RUNNING"
        assert phase == "DESCENDING"

    def test_sends_correct_command(self) -> None:
        """get_state() must send 'CMD GET_STATE'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("STATE IDLE NONE")
        make_ci(fake).get_state()
        assert fake.sent[0] == "CMD GET_STATE"

    def test_raises_on_malformed_response(self) -> None:
        """get_state() must raise CommandError if the response is not 'STATE x y'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK SOMETHING_UNEXPECTED")
        with pytest.raises(CommandError):
            make_ci(fake).get_state()

    def test_raises_on_timeout(self) -> None:
        """get_state() must raise CommandError when no response arrives."""
        fake: _FakeManager = _FakeManager()
        with pytest.raises(CommandError, match="Timeout"):
            make_ci(fake, ack_timeout_s=0.05).get_state()


# ---------------------------------------------------------------------------
# JOG
# ---------------------------------------------------------------------------

class TestJog:
    """CMD JOG direction validation and command string format."""

    def test_jog_up_correct_command(self) -> None:
        """jog('UP', ...) must send 'CMD JOG UP <speed>'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK JOG")
        make_ci(fake).jog("UP", 5.0)
        assert fake.sent[0] == "CMD JOG UP 5.0000"

    def test_jog_down_correct_command(self) -> None:
        """jog('DOWN', ...) must send 'CMD JOG DOWN <speed>'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK JOG")
        make_ci(fake).jog("DOWN", 5.0)
        assert fake.sent[0] == "CMD JOG DOWN 5.0000"

    def test_jog_with_accel_included(self) -> None:
        """When accel_mm_s2 > 0, the accel value must appear in the command."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK JOG")
        make_ci(fake).jog("UP", 5.0, accel_mm_s2=25.0)
        assert fake.sent[0] == "CMD JOG UP 5.0000 25.0000"

    def test_jog_without_accel_omitted(self) -> None:
        """When accel_mm_s2 is 0 (default), it must be omitted from the command."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK JOG")
        make_ci(fake).jog("DOWN", 3.5)
        sent: str = fake.sent[0]
        # Must have exactly 3 space-separated tokens after CMD.
        assert sent.count(" ") == 3   # "CMD JOG DOWN 3.5000"

    def test_invalid_direction_raises_value_error(self) -> None:
        """An unrecognised direction string must raise ValueError, not CommandError."""
        fake: _FakeManager = _FakeManager()
        with pytest.raises(ValueError, match="direction"):
            make_ci(fake).jog("LEFT", 5.0)

    def test_speed_formatted_to_4dp(self) -> None:
        """Speed must be formatted to exactly 4 decimal places."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK JOG")
        make_ci(fake).jog("UP", 7.5)
        assert "7.5000" in fake.sent[0]


# ---------------------------------------------------------------------------
# SET_TELEM_RATE and SET_SOFT_LIMITS
# ---------------------------------------------------------------------------

class TestSetTelemRate:
    """CMD SET_TELEM_RATE command string."""

    def test_sends_correct_command(self) -> None:
        """set_telem_rate(10) must send 'CMD SET_TELEM_RATE 10'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK SET_TELEM_RATE")
        make_ci(fake).set_telem_rate(10)
        assert fake.sent[0] == "CMD SET_TELEM_RATE 10"

    def test_zero_rate_allowed(self) -> None:
        """set_telem_rate(0) must send 'CMD SET_TELEM_RATE 0' (disables telemetry)."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK SET_TELEM_RATE")
        make_ci(fake).set_telem_rate(0)
        assert fake.sent[0] == "CMD SET_TELEM_RATE 0"


class TestSetSoftLimits:
    """CMD SET_SOFT_LIMITS command string and float formatting."""

    def test_sends_correct_command(self) -> None:
        """set_soft_limits must format both values to 4 decimal places."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK SET_SOFT_LIMITS")
        make_ci(fake).set_soft_limits(-5.0, 200.0)
        assert fake.sent[0] == "CMD SET_SOFT_LIMITS -5.0000 200.0000"

    def test_raises_on_err(self) -> None:
        """set_soft_limits must raise CommandError if Arduino replies ERR."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ERR SET_SOFT_LIMITS min_must_be_less_than_max")
        with pytest.raises(CommandError):
            make_ci(fake).set_soft_limits(100.0, -5.0)


# ---------------------------------------------------------------------------
# run() — trapezoidal dispatch
# ---------------------------------------------------------------------------

class TestRunTrapezoidal:
    """run() with a trapezoidal profile sends a single CMD RUN_PROFILE."""

    def test_sends_single_command(self) -> None:
        """run() with trapezoidal profile must send exactly one command."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK RUN_PROFILE")
        make_ci(fake).run(make_trapezoidal_profile())
        assert len(fake.sent) == 1

    def test_command_prefix(self) -> None:
        """The single command must start with 'CMD RUN_PROFILE'."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK RUN_PROFILE")
        make_ci(fake).run(make_trapezoidal_profile())
        assert fake.sent[0].startswith("CMD RUN_PROFILE")

    def test_command_contains_all_params(self) -> None:
        """CMD RUN_PROFILE must embed all 7 profile parameters."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK RUN_PROFILE")
        profile: DipProfile = make_trapezoidal_profile(
            dip_speed_mm_s      = 10.0,
            withdraw_speed_mm_s = 15.0,
            accel_mm_s2         = 50.0,
            dip_depth_mm        = 100.0,
            dwell_bottom_ms     = 1000,
            dwell_top_ms        = 500,
            n_dips              = 3,
        )
        make_ci(fake).run(profile)
        cmd: str = fake.sent[0]
        assert "10.0000" in cmd      # dip_speed_mm_s
        assert "15.0000" in cmd      # withdraw_speed_mm_s
        assert "50.0000" in cmd      # accel_mm_s2
        assert "100.0000" in cmd     # dip_depth_mm
        assert "1000" in cmd         # dwell_bottom_ms
        assert "500" in cmd          # dwell_top_ms
        assert cmd.endswith("3")     # n_dips

    def test_raises_on_err(self) -> None:
        """run() must raise CommandError when the Arduino rejects RUN_PROFILE."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ERR RUN_PROFILE invalid_state")
        with pytest.raises(CommandError):
            make_ci(fake).run(make_trapezoidal_profile())


# ---------------------------------------------------------------------------
# run() — segmented dispatch
# ---------------------------------------------------------------------------

class TestRunSegmented:
    """run() with a segmented profile streams the full protocol sequence."""

    def test_command_order(self) -> None:
        """
        Commands must arrive in the correct protocol order:
        BEGIN_SEGMENTED_MOVE → segments → RUN_LOADED_MOVE.
        """
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        profile: DipProfile = make_segmented_profile()
        make_ci(fake).run(profile)

        assert fake.sent[0].startswith("CMD BEGIN_SEGMENTED_MOVE")
        assert fake.sent[-1] == "CMD RUN_LOADED_MOVE"

    def test_begin_contains_segment_count(self) -> None:
        """BEGIN_SEGMENTED_MOVE must include the exact segment count."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        profile: DipProfile = make_segmented_profile()  # 3 segments
        make_ci(fake).run(profile)
        assert fake.sent[0] == "CMD BEGIN_SEGMENTED_MOVE 3"

    def test_move_seg_format(self) -> None:
        """CMD MOVE_SEG must include distance, speed, and accel formatted to 4dp."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        profile: DipProfile = make_segmented_profile()
        make_ci(fake).run(profile)

        # sent[1] is the first segment — a MOVE_SEG with distance=-50, speed=10, accel=50.
        move_cmd: str = fake.sent[1]
        assert move_cmd == "CMD MOVE_SEG -50.0000 10.0000 50.0000"

    def test_dwell_seg_format(self) -> None:
        """CMD DWELL_SEG must include the integer duration in milliseconds."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        profile: DipProfile = make_segmented_profile()
        make_ci(fake).run(profile)

        # sent[2] is the second segment — a DWELL_SEG with duration=1000.
        dwell_cmd: str = fake.sent[2]
        assert dwell_cmd == "CMD DWELL_SEG 1000"

    def test_total_command_count(self) -> None:
        """
        Total command count must be: 1 (BEGIN) + n_segs + 1 (RUN_LOADED_MOVE).
        """
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        profile: DipProfile = make_segmented_profile()  # 3 segments
        make_ci(fake).run(profile)

        expected_count: int = 1 + 3 + 1   # BEGIN + 3 segs + RUN_LOADED_MOVE
        assert len(fake.sent) == expected_count

    def test_move_seg_without_accel_omits_accel(self) -> None:
        """A move segment with no accel_mm_s2 key must send only dist and speed."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ACK BEGIN_SEGMENTED_MOVE")
        fake.response_queue.put("ACK PROFILE_READY")
        fake.response_queue.put("ACK RUN_LOADED_MOVE")

        segments: list[dict[str, Any]] = [
            {"type": "move", "distance_mm": 10.0, "speed_mm_s": 5.0},
        ]
        profile: DipProfile = make_segmented_profile(segments=segments)
        make_ci(fake).run(profile)

        move_cmd: str = fake.sent[1]
        assert move_cmd == "CMD MOVE_SEG 10.0000 5.0000"

    def test_zero_segments_raises(self) -> None:
        """An empty segments list must raise ValueError before any command is sent."""
        fake: _FakeManager = _FakeManager()

        segments_empty: list[dict[str, Any]] = []
        with pytest.raises(ValueError, match="no segments"):
            make_ci(fake).run(make_segmented_profile(segments=segments_empty))

        # No command must have been sent.
        assert fake.sent == []

    def test_too_many_segments_raises(self) -> None:
        """A segment list exceeding MOVE_SEG_BUFFER_SIZE must raise ValueError."""
        fake: _FakeManager = _FakeManager()

        oversized: list[dict[str, Any]] = [
            {"type": "dwell", "duration_ms": 100}
        ] * (MOVE_SEG_BUFFER_SIZE + 1)

        with pytest.raises(ValueError, match="exceeds"):
            make_ci(fake).run(make_segmented_profile(segments=oversized))

        assert fake.sent == []

    def test_unknown_segment_type_raises(self) -> None:
        """
        A segment dict with an unknown 'type' must raise ValueError.

        The profile validator in ``core/profile.py`` catches this at
        construction time, before any command is sent.  The error message
        comes from the validator, not from ``_run_segmented``.
        """
        fake: _FakeManager = _FakeManager()

        segments: list[dict[str, Any]] = [
            {"type": "spin", "rpm": 1000},
        ]
        with pytest.raises(ValueError, match="'spin'"):
            make_ci(fake).run(make_segmented_profile(segments=segments))

        # Profile construction failed — no commands should have been sent.
        assert fake.sent == []


# ---------------------------------------------------------------------------
# run() — spline and unknown dispatch
# ---------------------------------------------------------------------------

class TestRunDispatch:
    """run() dispatch for unsupported and unknown profile types."""

    def test_spline_raises_not_implemented(self) -> None:
        """run() with velocity_profile_type='spline' must raise NotImplementedError."""
        fake: _FakeManager = _FakeManager()
        profile: DipProfile = DipProfile(
            name                  = "spline_test",
            dip_speed_mm_s        = 10.0,
            withdraw_speed_mm_s   = 15.0,
            accel_mm_s2           = 50.0,
            dip_depth_mm          = 100.0,
            dwell_bottom_ms       = 0,
            dwell_top_ms          = 0,
            n_dips                = 1,
            velocity_profile_type = "spline",
        )
        with pytest.raises(NotImplementedError):
            make_ci(fake).run(profile)


# ---------------------------------------------------------------------------
# ACK/ERR handling
# ---------------------------------------------------------------------------

class TestAckHandling:
    """_wait_ack correctly handles ACK, ERR, and unexpected lines."""

    def test_stale_state_line_discarded(self) -> None:
        """
        A stale 'STATE ...' line before the expected ACK must be discarded
        and the real ACK still consumed.
        """
        fake: _FakeManager = _FakeManager()
        # Inject a stale STATE line followed by the real ACK.
        fake.response_queue.put("STATE READY NONE")
        fake.response_queue.put("ACK HOME")
        # home() must succeed despite the unexpected STATE line.
        make_ci(fake).home()

    def test_err_message_included_in_exception(self) -> None:
        """The CommandError message must contain the raw ERR line."""
        fake: _FakeManager = _FakeManager()
        fake.response_queue.put("ERR PAUSE invalid_state")
        with pytest.raises(CommandError, match="ERR PAUSE invalid_state"):
            make_ci(fake).pause()
