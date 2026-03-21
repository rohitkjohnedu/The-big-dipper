"""
tests/core/test_telemetry_parser.py
====================================

Tests for ``core/telemetry_parser.py``.

Covers:
- Happy path: all fields parsed and typed correctly.
- All valid state and phase enumeration values.
- Boundary values: zero timestamp, max uint32 timestamp, negative position/velocity.
- Malformed input: wrong field count, bad numerics, unknown state/phase, garbage lines.

All tests run without hardware — no serial port is required.
"""

from typing import Final

import pytest

from core.telemetry_parser import TelemetryFrame, parse

# ---------------------------------------------------------------------------
# Module-level constant
# ---------------------------------------------------------------------------

# A well-formed TELEM line used as the baseline for happy-path tests.
VALID_LINE: Final[str] = "TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING"


# ---------------------------------------------------------------------------
# Happy path — valid line parses correctly
# ---------------------------------------------------------------------------

class TestValidLine:
    """Parse a well-formed TELEM line and verify every field."""

    def test_returns_telemetry_frame(self) -> None:
        """parse() must return a TelemetryFrame instance, not None."""
        result: TelemetryFrame | None = parse(VALID_LINE)
        assert isinstance(result, TelemetryFrame)

    def test_timestamp(self) -> None:
        """timestamp_ms must be parsed as an integer."""
        expected_timestamp: int = 12453
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.timestamp_ms == expected_timestamp

    def test_position(self) -> None:
        """pos_mm must be parsed as a float with correct value."""
        expected_pos: float = 45.32
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.pos_mm == pytest.approx(expected_pos)

    def test_actual_velocity(self) -> None:
        """vel_actual_mm_s must be parsed as a float."""
        expected_vel: float = 10.00
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.vel_actual_mm_s == pytest.approx(expected_vel)

    def test_commanded_velocity(self) -> None:
        """vel_commanded_mm_s must be parsed as a float."""
        expected_vel: float = 10.00
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.vel_commanded_mm_s == pytest.approx(expected_vel)

    def test_accel(self) -> None:
        """accel_mm_s2 must be parsed as a float (currently always 0.00)."""
        expected_accel: float = 0.00
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.accel_mm_s2 == pytest.approx(expected_accel)

    def test_state(self) -> None:
        """state must be returned as a string."""
        expected_state: str = "RUNNING"
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.state == expected_state

    def test_phase(self) -> None:
        """phase must be returned as a string."""
        expected_phase: str = "DESCENDING"
        frame: TelemetryFrame | None = parse(VALID_LINE)
        assert frame is not None
        assert frame.phase == expected_phase

    def test_trailing_newline_stripped(self) -> None:
        """Lines ending with \\n (Unix line ending) must parse successfully."""
        result: TelemetryFrame | None = parse(VALID_LINE + "\n")
        assert result is not None

    def test_trailing_crlf_stripped(self) -> None:
        """Lines ending with \\r\\n (Windows line ending) must parse successfully."""
        result: TelemetryFrame | None = parse(VALID_LINE + "\r\n")
        assert result is not None


# ---------------------------------------------------------------------------
# All valid states and phases
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("state", ["IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"])
def test_all_valid_states(state: str) -> None:
    """Every state string defined in the Arduino firmware must be accepted."""
    line: str = f"TELEM,0,0.00,0.00,0.00,0.00,{state},NONE"
    frame: TelemetryFrame | None = parse(line)
    assert frame is not None
    assert frame.state == state


@pytest.mark.parametrize("phase", ["NONE", "DESCENDING", "DWELL_BOTTOM", "ASCENDING", "DWELL_TOP"])
def test_all_valid_phases(phase: str) -> None:
    """Every phase string defined in the Arduino firmware must be accepted."""
    line: str = f"TELEM,0,0.00,0.00,0.00,0.00,READY,{phase}"
    frame: TelemetryFrame | None = parse(line)
    assert frame is not None
    assert frame.phase == phase


# ---------------------------------------------------------------------------
# Boundary values
# ---------------------------------------------------------------------------

def test_zero_timestamp() -> None:
    """timestamp_ms of 0 is valid (first broadcast after startup)."""
    expected_timestamp: int = 0
    frame: TelemetryFrame | None = parse("TELEM,0,0.00,0.00,0.00,0.00,IDLE,NONE")
    assert frame is not None
    assert frame.timestamp_ms == expected_timestamp


def test_large_timestamp() -> None:
    """
    timestamp_ms near the uint32 max (2^32 - 1 = 4294967295) must be accepted.

    Arduino millis() rolls over to 0 after ~49 days.  The parser must handle
    values up to the full uint32 range without overflow or rejection.
    """
    expected_timestamp: int = 4294967295
    frame: TelemetryFrame | None = parse(
        "TELEM,4294967295,0.00,0.00,0.00,0.00,IDLE,NONE"
    )
    assert frame is not None
    assert frame.timestamp_ms == expected_timestamp


def test_negative_position() -> None:
    """
    Negative pos_mm is valid and common — it means the carriage is below home,
    i.e. dipped into the coating solution.
    """
    expected_pos: float = -50.00
    frame: TelemetryFrame | None = parse(
        "TELEM,100,-50.00,5.00,5.00,0.00,RUNNING,DESCENDING"
    )
    assert frame is not None
    assert frame.pos_mm == pytest.approx(expected_pos)


def test_negative_velocity() -> None:
    """Negative velocity is valid — it indicates downward motion."""
    expected_vel: float = -10.00
    frame: TelemetryFrame | None = parse(
        "TELEM,100,0.00,-10.00,-10.00,0.00,RUNNING,DESCENDING"
    )
    assert frame is not None
    assert frame.vel_actual_mm_s == pytest.approx(expected_vel)


# ---------------------------------------------------------------------------
# Malformed input — must return None, never raise
# ---------------------------------------------------------------------------

class TestMalformedInput:
    """
    Verify that parse() degrades gracefully on any malformed input.

    The contract is: return None and log a warning.  Never raise an exception.
    Callers in the serial read loop rely on this guarantee.
    """

    def test_empty_string(self) -> None:
        """An empty string has no TELEM prefix and must return None."""
        result: TelemetryFrame | None = parse("")
        assert result is None

    def test_wrong_prefix(self) -> None:
        """Non-TELEM lines (ACK, ERR, etc.) must return None silently."""
        result: TelemetryFrame | None = parse("ACK HOME")
        assert result is None

    def test_too_few_fields(self) -> None:
        """A line with fewer than 8 comma-separated fields must return None."""
        result: TelemetryFrame | None = parse("TELEM,12453,45.32,10.00")
        assert result is None

    def test_too_many_fields(self) -> None:
        """A line with more than 8 fields must return None."""
        result: TelemetryFrame | None = parse(VALID_LINE + ",extra")
        assert result is None

    def test_non_numeric_timestamp(self) -> None:
        """A non-integer timestamp field must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,abc,45.32,10.00,10.00,0.00,RUNNING,DESCENDING"
        )
        assert result is None

    def test_non_numeric_position(self) -> None:
        """A non-float position field must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,100,xyz,10.00,10.00,0.00,RUNNING,DESCENDING"
        )
        assert result is None

    def test_unknown_state(self) -> None:
        """A state string not in the known set must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,100,0.00,0.00,0.00,0.00,UNKNOWN,NONE"
        )
        assert result is None

    def test_unknown_phase(self) -> None:
        """A phase string not in the known set must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,100,0.00,0.00,0.00,0.00,READY,UNKNOWN"
        )
        assert result is None

    def test_empty_state(self) -> None:
        """An empty state field (two consecutive commas) must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,100,0.00,0.00,0.00,0.00,,NONE"
        )
        assert result is None

    def test_empty_phase(self) -> None:
        """An empty phase field (trailing comma) must return None."""
        result: TelemetryFrame | None = parse(
            "TELEM,100,0.00,0.00,0.00,0.00,READY,"
        )
        assert result is None

    def test_does_not_raise_on_garbage(self) -> None:
        """
        Completely unrelated input must return None without raising.

        This is the most important guarantee — the serial read loop calls
        parse() on every line and must never crash from unexpected input.
        """
        result: TelemetryFrame | None
        try:
            result = parse("not a telem line at all !!!")
            assert result is None
        except Exception as exc:
            pytest.fail(f"parse() raised unexpectedly: {exc}")


# ---------------------------------------------------------------------------
# Non-finite float values — must be rejected
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("bad_value", ["inf", "-inf", "nan", "Infinity", "-Infinity"])
def test_non_finite_float_rejected(bad_value: str) -> None:
    """
    Float fields containing inf, -inf, or nan must return None.

    The Arduino never produces non-finite values; their presence indicates
    a corrupt serial frame.  Python's float() happily converts "inf" and
    "nan" strings, so the parser must explicitly guard against them.
    """
    line: str = f"TELEM,100,{bad_value},0.00,0.00,0.00,RUNNING,DESCENDING"
    result: TelemetryFrame | None = parse(line)
    assert result is None
