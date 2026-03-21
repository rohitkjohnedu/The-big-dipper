"""
core/telemetry_parser.py
========================

Parses raw TELEM lines received from the Arduino serial stream into typed
``TelemetryFrame`` dataclass instances.

All serial-to-Python conversion lives here so the rest of the application
works with structured objects rather than raw strings.  Any line that does
not conform to the expected format is silently rejected (returns ``None`` and
logs a warning) — the caller never needs to handle parse exceptions.

Wire format (defined in arduino/docs/TELEMETRY.md)::

    TELEM,<millis_ms>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>

Example::

    TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING
"""

import logging
import math
from dataclasses import dataclass
from typing import Final

# Module-level logger — messages appear under the name "core.telemetry_parser"
# so they can be filtered independently from other modules.
log: Final[logging.Logger] = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Valid enumeration values for the state and phase fields.
# These mirror the string representations produced by stateString() and
# phaseString() in the Arduino state_machine.cpp.  Any value outside these
# sets indicates a garbled or firmware-version-mismatched line.
# ---------------------------------------------------------------------------

_VALID_STATES: Final[frozenset[str]] = frozenset({
    "IDLE",
    "HOMING",
    "READY",
    "RUNNING",
    "PAUSED",
    "ERROR",
})

_VALID_PHASES: Final[frozenset[str]] = frozenset({
    "NONE",
    "DESCENDING",
    "DWELL_BOTTOM",
    "ASCENDING",
    "DWELL_TOP",
})

# Total number of comma-separated fields in a valid TELEM line,
# including the leading "TELEM" prefix token.
_EXPECTED_FIELD_COUNT: Final[int] = 8


# ---------------------------------------------------------------------------
# Public dataclass
# ---------------------------------------------------------------------------

@dataclass
class TelemetryFrame:
    """
    One snapshot of machine state as broadcast by the Arduino telemetry module.

    Fields are named to match the wire format exactly so that a TelemetryFrame
    can be logged to CSV without any renaming.  All linear values are in mm,
    mm/s, or mm/s² as received — no unit conversion is performed here.

    Attributes:
        timestamp_ms:       Arduino ``millis()`` value at the moment the line
                            was broadcast.  Rolls over to 0 after ~49 days.
        pos_mm:             Encoder position relative to the home endstop.
                            Negative values mean the carriage is below home
                            (i.e. dipped into the coating solution).
        vel_actual_mm_s:    Shaft velocity measured from the encoder RPM.
                            Negative indicates downward motion.
        vel_commanded_mm_s: Last velocity setpoint written to the stepper
                            driver.  May differ from ``vel_actual_mm_s``
                            during acceleration/deceleration ramps.
        accel_mm_s2:        Actual acceleration derived from encoder data.
                            Currently always ``0.00`` — not yet implemented
                            on the firmware side.
        state:              System state string.  One of:
                            ``IDLE | HOMING | READY | RUNNING | PAUSED | ERROR``
        phase:              Run sub-phase string.  One of:
                            ``NONE | DESCENDING | DWELL_BOTTOM | ASCENDING | DWELL_TOP``
    """

    timestamp_ms:       int    # Arduino millis() at broadcast time (ms)
    pos_mm:             float  # Encoder position relative to home (mm)
    vel_actual_mm_s:    float  # Encoder-measured shaft velocity (mm/s)
    vel_commanded_mm_s: float  # Last velocity setpoint sent to driver (mm/s)
    accel_mm_s2:        float  # Actual acceleration — always 0.00 for now (mm/s²)
    state:              str    # System state: IDLE | HOMING | READY | RUNNING | PAUSED | ERROR
    phase:              str    # Run phase:  NONE | DESCENDING | DWELL_BOTTOM | ASCENDING | DWELL_TOP


# ---------------------------------------------------------------------------
# Public parsing function
# ---------------------------------------------------------------------------

def parse(line: str) -> TelemetryFrame | None:
    """
    Parse a raw TELEM line into a ``TelemetryFrame``.

    This function is the single entry point for all telemetry parsing.  It is
    designed to be called in a tight serial read loop, so it is intentionally
    defensive: any malformed input returns ``None`` and emits a warning rather
    than raising an exception.

    Validation steps performed in order:

    1. Strip leading/trailing whitespace (handles ``\\n``, ``\\r\\n``).
    2. Check the ``TELEM,`` prefix — non-TELEM lines return ``None`` silently
       (they are normal; ACK/ERR lines are routed elsewhere by SerialManager).
    3. Count comma-separated fields — must be exactly ``_EXPECTED_FIELD_COUNT``.
    4. Convert numeric fields via ``int()`` / ``float()`` — rejects non-numeric
       tokens.
    5. Validate ``state`` and ``phase`` against their known enumeration sets.

    Args:
        line: Raw string received from the serial port, with or without a
              trailing newline or carriage-return character.

    Returns:
        A fully populated ``TelemetryFrame`` on success, or ``None`` if the
        line is not a TELEM line or fails any validation step.

    Examples::

        frame = parse("TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING")
        assert frame is not None
        assert frame.state == "RUNNING"

        assert parse("ACK HOME") is None          # non-TELEM line
        assert parse("TELEM,bad,line") is None    # conversion failure
    """

    # --- Step 1: strip whitespace (handles \n, \r\n, leading spaces) ---------
    line = line.strip()

    # --- Step 2: quick prefix check ------------------------------------------
    # Return None silently — non-TELEM lines are expected (ACK, ERR, STATE, …)
    # and handled by a different part of the system.
    if not line.startswith("TELEM,"):
        return None

    # --- Step 3: split and count fields --------------------------------------
    parts: list[str] = line.split(",")

    if len(parts) != _EXPECTED_FIELD_COUNT:
        log.warning(
            "telemetry_parser: expected %d fields, got %d in %r",
            _EXPECTED_FIELD_COUNT, len(parts), line,
        )
        return None

    # --- Step 4: convert numeric fields --------------------------------------
    # Each conversion is wrapped so a single bad field emits one clear warning.
    try:
        timestamp_ms:       int   = int(parts[1])
        pos_mm:             float = float(parts[2])
        vel_actual_mm_s:    float = float(parts[3])
        vel_commanded_mm_s: float = float(parts[4])
        accel_mm_s2:        float = float(parts[5])
    except ValueError as exc:
        log.warning("telemetry_parser: numeric conversion error in %r: %s", line, exc)
        return None

    # Reject non-finite float values (inf, -inf, nan).  The Arduino never
    # produces these, so their presence indicates a corrupt serial frame.
    _floats: list[float] = [pos_mm, vel_actual_mm_s, vel_commanded_mm_s, accel_mm_s2]
    if any(not math.isfinite(v) for v in _floats):
        log.warning("telemetry_parser: non-finite float value in %r", line)
        return None

    # --- Step 5: validate string enumeration fields --------------------------
    state: str = parts[6]
    phase: str = parts[7]

    if state not in _VALID_STATES:
        log.warning("telemetry_parser: unknown state %r in %r", state, line)
        return None

    if phase not in _VALID_PHASES:
        log.warning("telemetry_parser: unknown phase %r in %r", phase, line)
        return None

    # --- All checks passed — construct and return the frame ------------------
    return TelemetryFrame(
        timestamp_ms       = timestamp_ms,
        pos_mm             = pos_mm,
        vel_actual_mm_s    = vel_actual_mm_s,
        vel_commanded_mm_s = vel_commanded_mm_s,
        accel_mm_s2        = accel_mm_s2,
        state              = state,
        phase              = phase,
    )
