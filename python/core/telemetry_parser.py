"""
core/telemetry_parser.py

Parses raw TELEM lines from the Arduino serial stream into TelemetryFrame
dataclass instances.  All parsing is done in one place so the rest of the
application works with typed objects, not raw strings.

Wire format (from TELEMETRY.md):
    TELEM,<millis_ms>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>

Example:
    TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING
"""

import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)

# Valid values for the state and phase fields — used to catch garbled lines.
_VALID_STATES = {"IDLE", "HOMING", "READY", "RUNNING", "PAUSED", "ERROR"}
_VALID_PHASES = {"NONE", "DESCENDING", "DWELL_BOTTOM", "ASCENDING", "DWELL_TOP"}


@dataclass
class TelemetryFrame:
    """One snapshot of machine state broadcast by the Arduino."""
    timestamp_ms:       int    # Arduino millis() at broadcast time
    pos_mm:             float  # Encoder position relative to home (negative = below home)
    vel_actual_mm_s:    float  # Shaft velocity measured from encoder RPM
    vel_commanded_mm_s: float  # Last velocity written to the stepper driver
    accel_mm_s2:        float  # Actual acceleration (always 0.00 — not yet implemented)
    state:              str    # System state: IDLE | HOMING | READY | RUNNING | PAUSED | ERROR
    phase:              str    # Run phase: NONE | DESCENDING | DWELL_BOTTOM | ASCENDING | DWELL_TOP


def parse(line: str) -> TelemetryFrame | None:
    """
    Parse a raw TELEM line into a TelemetryFrame.

    Returns None (and logs a warning) on any malformed input so the caller
    never has to handle exceptions from this function.

    Args:
        line: Raw string from the serial stream, with or without trailing newline.

    Returns:
        TelemetryFrame on success, None on any parse failure.
    """
    line = line.strip()

    if not line.startswith("TELEM,"):
        return None

    parts = line.split(",")

    if len(parts) != 8:
        log.warning("telemetry_parser: expected 8 fields, got %d: %r", len(parts), line)
        return None

    try:
        timestamp_ms       = int(parts[1])
        pos_mm             = float(parts[2])
        vel_actual_mm_s    = float(parts[3])
        vel_commanded_mm_s = float(parts[4])
        accel_mm_s2        = float(parts[5])
        state              = parts[6]
        phase              = parts[7]
    except ValueError as exc:
        log.warning("telemetry_parser: conversion error in %r: %s", line, exc)
        return None

    if state not in _VALID_STATES:
        log.warning("telemetry_parser: unknown state %r in %r", state, line)
        return None

    if phase not in _VALID_PHASES:
        log.warning("telemetry_parser: unknown phase %r in %r", phase, line)
        return None

    return TelemetryFrame(
        timestamp_ms       = timestamp_ms,
        pos_mm             = pos_mm,
        vel_actual_mm_s    = vel_actual_mm_s,
        vel_commanded_mm_s = vel_commanded_mm_s,
        accel_mm_s2        = accel_mm_s2,
        state              = state,
        phase              = phase,
    )
