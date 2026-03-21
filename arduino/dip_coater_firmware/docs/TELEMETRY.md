# Telemetry Module

`telemetry.h` / `telemetry.cpp`

---

## Overview

The Telemetry module streams machine state to the Python UI over Serial at a
configurable rate (default `DEFAULT_TELEM_RATE_HZ`, max `MAX_TELEM_RATE_HZ`).
It is entirely **non-blocking** — `update()` checks an elapsed-time gate every
`loop()` iteration and only writes to Serial when an interval has expired.

`Telemetry` does **not** own any hardware.  It reads from `MotionController`
and `StateMachine` through their public accessors and formats the result as a
single CSV line.

---

## Output Format

```
TELEM,<millis_ms>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>
```

| Field | Type | Description |
|---|---|---|
| `millis_ms` | uint32 | Arduino `millis()` timestamp at broadcast time |
| `pos_mm` | float, 2 dp | Encoder position relative to home (negative = below home) |
| `vel_actual_mm_s` | float, 2 dp | Shaft velocity measured from encoder RPM |
| `vel_commanded_mm_s` | float, 2 dp | Last velocity written to the stepper driver |
| `accel_mm_s2` | float, 2 dp | Actual acceleration — not yet implemented, always `0.00` |
| `state` | string | System state: `IDLE`, `HOMING`, `READY`, `RUNNING`, `PAUSED`, `ERROR` |
| `phase` | string | Run phase: `NONE`, `DESCENDING`, `DWELL_BOTTOM`, `ASCENDING`, `DWELL_TOP` |

**Example:**
```
TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING
```

---

## Why both actual and commanded velocity?

The TMC5130 runs a closed-loop PID.  During acceleration and deceleration ramps
the actual velocity lags behind the commanded target.  Broadcasting both allows
the Python data recorder to verify profile fidelity post-run — if they diverge
significantly the profile parameters may need tuning.

---

## Rate control

```
CMD SET_TELEM_RATE <hz>    # 0 = off, 1–50 Hz accepted
```

`CommandParser` validates the range and calls `Telemetry::setRate()`.  The rate
can be changed at any time including during a running profile.

| Rate | Interval | Typical use |
|---|---|---|
| 0 | disabled | No streaming needed |
| 1 Hz | 1000 ms | Default — light serial load, basic monitoring |
| 10 Hz | 100 ms | Recommended for live plotting in the Python UI |
| 50 Hz | 20 ms | Maximum — use only when logging high-speed profiles |

---

## Timing implementation

`update()` uses an **anchor-based** interval rather than a simple `now - last >= interval` check:

```
_lastBroadcastMs += _intervalMs;   // advance anchor by one slot
```

This prevents slow drift from accumulating loop-jitter over many broadcasts.
If the firmware falls more than one full interval behind (e.g. a long ISR),
it resyncs to `now` to avoid a burst of catch-up broadcasts.

Unsigned arithmetic is used throughout so the `millis()` 49-day rollover is
handled correctly with no special-case code.

---

## Wiring (`.ino`)

```cpp
// Global objects
Telemetry telem(mc, sm);

// setup()
telem.begin();
commandParser.setTelemetry(&telem);   // wires CMD SET_TELEM_RATE

// loop()
telem.update();
```

`CommandParser` holds a pointer to `Telemetry` (set via `setTelemetry()`).
The pointer is checked before use so the firmware compiles and runs correctly
even if `setTelemetry()` is never called.

---

## Constants

| Constant | Value | Meaning |
|---|---|---|
| `DEFAULT_TELEM_RATE_HZ` | 1 | Rate on startup |
| `MAX_TELEM_RATE_HZ` | 50 | Hard cap applied in `setRate()` |
