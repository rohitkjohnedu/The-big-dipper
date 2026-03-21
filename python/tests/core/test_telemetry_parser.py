"""
tests/core/test_telemetry_parser.py

Tests for core/telemetry_parser.py.
All tests run without hardware — no serial port required.
"""

import pytest
from core.telemetry_parser import TelemetryFrame, parse

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

VALID_LINE = "TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING"


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------

class TestValidLine:
    def test_returns_telemetry_frame(self):
        assert isinstance(parse(VALID_LINE), TelemetryFrame)

    def test_timestamp(self):
        assert parse(VALID_LINE).timestamp_ms == 12453

    def test_position(self):
        assert parse(VALID_LINE).pos_mm == pytest.approx(45.32)

    def test_actual_velocity(self):
        assert parse(VALID_LINE).vel_actual_mm_s == pytest.approx(10.00)

    def test_commanded_velocity(self):
        assert parse(VALID_LINE).vel_commanded_mm_s == pytest.approx(10.00)

    def test_accel(self):
        assert parse(VALID_LINE).accel_mm_s2 == pytest.approx(0.00)

    def test_state(self):
        assert parse(VALID_LINE).state == "RUNNING"

    def test_phase(self):
        assert parse(VALID_LINE).phase == "DESCENDING"

    def test_trailing_newline_stripped(self):
        assert parse(VALID_LINE + "\n") is not None

    def test_trailing_crlf_stripped(self):
        assert parse(VALID_LINE + "\r\n") is not None


# ---------------------------------------------------------------------------
# All valid states and phases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("state", ["IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"])
def test_all_valid_states(state):
    line = f"TELEM,0,0.00,0.00,0.00,0.00,{state},NONE"
    frame = parse(line)
    assert frame is not None
    assert frame.state == state


@pytest.mark.parametrize("phase", ["NONE", "DESCENDING", "DWELL_BOTTOM", "ASCENDING", "DWELL_TOP"])
def test_all_valid_phases(phase):
    line = f"TELEM,0,0.00,0.00,0.00,0.00,READY,{phase}"
    frame = parse(line)
    assert frame is not None
    assert frame.phase == phase


# ---------------------------------------------------------------------------
# Boundary values
# ---------------------------------------------------------------------------

def test_zero_timestamp():
    assert parse("TELEM,0,0.00,0.00,0.00,0.00,IDLE,NONE").timestamp_ms == 0

def test_large_timestamp():
    # millis() rolls over after ~49 days (2^32 ms); test a value near that
    frame = parse("TELEM,4294967295,0.00,0.00,0.00,0.00,IDLE,NONE")
    assert frame.timestamp_ms == 4294967295

def test_negative_position():
    # Negative position = below home (dipped into solution)
    frame = parse("TELEM,100,-50.00,5.00,5.00,0.00,RUNNING,DESCENDING")
    assert frame.pos_mm == pytest.approx(-50.00)

def test_negative_velocity():
    frame = parse("TELEM,100,0.00,-10.00,-10.00,0.00,RUNNING,DESCENDING")
    assert frame.vel_actual_mm_s == pytest.approx(-10.00)


# ---------------------------------------------------------------------------
# Malformed input — must return None, never raise
# ---------------------------------------------------------------------------

class TestMalformedInput:
    def test_empty_string(self):
        assert parse("") is None

    def test_wrong_prefix(self):
        assert parse("ACK HOME") is None

    def test_too_few_fields(self):
        assert parse("TELEM,12453,45.32,10.00") is None

    def test_too_many_fields(self):
        assert parse(VALID_LINE + ",extra") is None

    def test_non_numeric_timestamp(self):
        assert parse("TELEM,abc,45.32,10.00,10.00,0.00,RUNNING,DESCENDING") is None

    def test_non_numeric_position(self):
        assert parse("TELEM,100,xyz,10.00,10.00,0.00,RUNNING,DESCENDING") is None

    def test_unknown_state(self):
        assert parse("TELEM,100,0.00,0.00,0.00,0.00,UNKNOWN,NONE") is None

    def test_unknown_phase(self):
        assert parse("TELEM,100,0.00,0.00,0.00,0.00,READY,UNKNOWN") is None

    def test_empty_state(self):
        assert parse("TELEM,100,0.00,0.00,0.00,0.00,,NONE") is None

    def test_empty_phase(self):
        assert parse("TELEM,100,0.00,0.00,0.00,0.00,READY,") is None

    def test_does_not_raise_on_garbage(self):
        # Must never raise — callers rely on None return
        try:
            result = parse("not a telem line at all !!!")
            assert result is None
        except Exception as exc:
            pytest.fail(f"parse() raised unexpectedly: {exc}")
