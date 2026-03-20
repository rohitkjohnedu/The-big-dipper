# Diagnostics Module

`diagnostics.h` / `diagnostics.cpp`

---

## Overview

The Diagnostics module provides hardware self-tests and a calibration utility.
It is driven by `DIAG` serial commands and runs all tests **non-blocking** through
its `update()` method — each test is a state machine that advances one step per
`loop()` iteration.

`Diagnostics` does **not** manage its own motor — it calls `MotionController`
methods for all motion.  It bypasses the `StateMachine` running/paused checks
(the motor is used in whatever state the system is in) but does check for
`ERROR` state after moves that might have been rejected by soft limits.

---

## Active Modes

```mermaid
stateDiagram-v2
    [*]          --> INACTIVE

    INACTIVE     --> MOTOR_TEST    : DIAG MOTOR / DIAG MOTORENCODER
    INACTIVE     --> ENDSTOP_TEST  : DIAG ENDSTOP
    INACTIVE     --> JOG           : DIAG JOG UP/DOWN
    INACTIVE     --> MOVE_TEST     : DIAG MOVE
    INACTIVE     --> CAL_MOVE      : DIAG CAL

    MOTOR_TEST   --> INACTIVE      : test done or DIAG EXIT
    ENDSTOP_TEST --> INACTIVE      : DIAG EXIT
    JOG          --> INACTIVE      : DIAG EXIT / DIAG JOG STOP
    MOVE_TEST    --> INACTIVE      : move complete or timeout
    CAL_MOVE     --> INACTIVE      : motor settled or timeout
```

---

## DIAG MOTOR / DIAG MOTORENCODER

Runs a simple two-leg move: down 5 mm then back up 5 mm.
`MOTORENCODER` variant also checks encoder accuracy against the commanded distance.

```mermaid
flowchart TD
    A(["startMotorTest / startMotorEncoderTest"]) --> B["motorPhase = MOVE_DOWN<br/>moveByMm(+5mm, speed, accel)"]

    B --> C["MOVE_DOWN phase:<br/>wait for motor to leave start<br/>MOVE_START_MM = 0.2 mm"]
    C --> D{"position moved >= MOVE_START_MM?"}
    D -- No --> C
    D -- Yes --> E["track position stability<br/>STABLE_MM = 0.05 mm over SETTLE_MS = 300 ms"]
    E --> F{stable for 300 ms?}
    F -- No --> E
    F -- Yes --> G{checkEncoder?}
    G -- Yes --> H["check Move down:<br/>|actual - 5mm| <= 1.5mm"]
    G -- No  --> I["print move down complete"]
    H --> J
    I --> J["motorPhase = MOVE_UP<br/>moveByMm(-5mm, speed, accel)"]

    J --> K["MOVE_UP phase:<br/>same stability detection"]
    K --> L{stable for 300 ms?}
    L -- No --> K
    L -- Yes --> M{checkEncoder?}
    M -- Yes --> N["check Move up:<br/>|actual - 5mm| <= 1.5mm"]
    M -- No  --> O["print move up complete"]
    N --> P
    O --> P["motorPhase = DONE"]

    P --> Q["print DONE with pass/fail counts<br/>stop motor<br/>mode = INACTIVE"]
```

**Completion detection** in `updateMotorTest()` uses a **position-stability heuristic**:
the motor is considered stopped when position changes by less than `STABLE_MM` (0.05 mm)
over `SETTLE_MS` (300 ms).  This is reliable at `DIAG_SPEED_MMS` (2 mm/s) but would
fail at speeds below ~0.17 mm/s — use `DIAG MOVE` for slow-speed testing.

---

## DIAG MOVE

`DIAG MOVE <mm> [speed_mm_s] [accel_mm_s2]`

A precise single-move test with PASS/FAIL output.  Uses the **STANDSTILL signal**
for completion detection instead of the position-stability heuristic, making it
reliable at any speed down to 0.05 mm/s (tested).

```mermaid
flowchart TD
    A(["startMoveTest called"]) --> B["store params<br/>record moveTestStartPos<br/>record moveTestStart<br/>moveByMm(mm, speed, accel)"]
    B --> C{"state == ERROR?<br/>soft limit hit?"}
    C -- Yes --> D(["mode = INACTIVE, silent abort"])
    C -- No  --> E["print DIAG:MOVE:START<br/>mode = MOVE_TEST"]

    E --> F["updateMoveTest:<br/>timeoutMs = max(60s, 3 x expected_travel_time)"]
    F --> G{"motor left startPos<br/>by >= 0.2 mm?"}
    G -- No, not timed out --> G
    G -- Yes --> H["moveStarted = true"]
    G -- timed out --> I

    H --> I{"isStandstill() == 0 (stopped)?<br/>or timed out?"}
    I -- No --> I
    I -- Yes --> J["actual = currentPos - startPos<br/>ok = |actual - commanded| <= 1.5 mm"]
    J --> K["print DIAG:MOVE:PASS or FAIL<br/>with commanded= and actual="]
    K --> L(["stop, mode = INACTIVE"])
```

**Why STANDSTILL instead of position stability?**

At slow speeds (e.g. 0.1 mm/s), the motor moves only 0.03 mm in 300 ms.
That is less than `STABLE_MM` (0.05 mm), so the stability heuristic would
declare the motor stopped before it has actually started moving.  The STANDSTILL
signal from the TMC5130 driver is independent of speed and is reliable at any
commanded velocity.

**Dynamic timeout:**
```
timeoutMs = max(60000,  3 x (|dist_mm| / speed_mm_s) x 1000)
```
This prevents false timeouts at very slow speeds — e.g. 10 mm at 0.1 mm/s
needs 100 s, which exceeds the 60 s fixed timeout.

---

## DIAG ENDSTOP

Live endstop monitor.  Prints each state change with a 20 ms debounce window.

```mermaid
flowchart TD
    A(["startEndstopTest"]) --> B["seed _lastTop, _lastBot<br/>from current pin state"]
    B --> C["updateEndstopTest:<br/>read rawTop, rawBot"]

    C --> D{"rawTop != _lastTop?"}
    D -- Yes, not pending --> E["pendingTop = true<br/>start debounce timer"]
    E --> F{"stable for 20 ms?"}
    F -- No, still same value --> F
    F -- Yes --> G["_lastTop = rawTop<br/>print DIAG:ENDSTOP:TOP:TRIGGERED or RELEASED"]
    D -- No --> H["pendingTop = false, was noise"]

    C --> I{"rawBot != _lastBot?"}
    I -- Yes, not pending --> J["pendingBot = true<br/>start debounce timer"]
    J --> K{"stable for 20 ms?"}
    K -- Yes --> L["print DIAG:ENDSTOP:BOTTOM:TRIGGERED or RELEASED"]
    I -- No --> M["pendingBot = false"]

    G --> C
    L --> C
    H --> C
    M --> C
```

> **Note:** While in `ENDSTOP_TEST` mode the ISR wrappers in the `.ino` file bypass
> `mc.onEndstopTriggered()` so manually pressing endstops does not trigger the
> limit-backoff logic.

---

## DIAG CAL (Calibration)

Used to measure and correct the `LEADSCREW_MM_PER_REV` constant in `config.h`.

```mermaid
sequenceDiagram
    participant U  as User
    participant FW as Firmware

    U->>FW: DIAG CAL 50
    FW-->>U: DIAG:CAL:START commanded=50.0mm at 10mm/s
    Note over FW: motor moves 50 mm, waits for position to stabilise
    FW-->>U: DIAG:CAL:DONE encoder=49.82mm
    FW-->>U: DIAG:CAL:Measure the actual displacement with calipers.
    FW-->>U: DIAG:CAL:Then type DIAG CAL RESULT actual_mm

    U->>FW: DIAG CAL RESULT 50.3
    FW-->>U: DIAG:CAL:RESULT commanded=50.0mm encoder=49.82mm actual=50.30mm
    FW-->>U: DIAG:CAL:correction=1.0096
    FW-->>U: DIAG:CAL:Update config.h LEADSCREW_MM_PER_REV = 8.0768
```

**Correction formula:**
```
correction     = actual_mm / encoder_mm
new_mm_per_rev = LEADSCREW_MM_PER_REV x correction
```

If the correction is within 1% of 1.0, no change is needed.

**Completion detection** in `updateCalMove()` uses the same position-stability
heuristic as `updateMotorTest()` (SETTLE_MS / STABLE_MM), since the calibration
move is always run at `CAL_SPEED_MMS` (10 mm/s) where stability detection is reliable.

---

## DIAG JOG

Starts continuous velocity motion for manual stage positioning.
The motor runs until `DIAG EXIT` or `DIAG JOG STOP` is received.

`update()` does nothing in JOG mode — `MotionController.jog()` uses
`runContinous()` which the library drives autonomously.

---

## Constants Summary

| Constant | Value | Meaning |
|---|---|---|
| `DIAG_SPEED_MMS` | 2.0 mm/s | Default speed for DIAG MOTOR and DIAG MOVE |
| `DIAG_ACCEL_MMS2` | 5.0 mm/s² | Default acceleration |
| `CAL_SPEED_MMS` | 10.0 mm/s | Calibration move speed |
| `DIAG_DIST_MM` | 5.0 mm | Distance for DIAG MOTOR test |
| `TOLERANCE_MM` | 1.5 mm | Max error for PASS (DIAG MOTORENCODER, DIAG MOVE) |
| `MOVE_TIMEOUT_MS` | 60 000 ms | Minimum move timeout (DIAG MOVE scales this up) |
| `SETTLE_MS` | 300 ms | Stability window for motor-test completion |
| `STABLE_MM` | 0.05 mm | Max position drift to count as settled |
| `MOVE_START_MM` | 0.2 mm | Position change needed to detect that movement has started |
| `DEBOUNCE_MS` | 20 ms | Endstop debounce window |
