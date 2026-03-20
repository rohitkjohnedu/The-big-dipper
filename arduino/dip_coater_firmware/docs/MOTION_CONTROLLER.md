# Motion Controller

`motion_controller.h` / `motion_controller.cpp`

The MotionController owns the UstepperS32 stepper driver instance and is the
**single place in the firmware that issues commands to the motor**.  All public
methods are non-blocking: call `begin()` once from `setup()` and `update()` on
every `loop()` iteration.

---

## Internal Motion Modes (ProfileMode)

The controller uses an internal `ProfileMode` enum to track what the motor is
currently doing.  `update()` dispatches to the appropriate handler each loop.

```mermaid
stateDiagram-v2
    [*]          --> NONE
    NONE         --> HOMING        : executeHome()
    NONE         --> JOG           : jog()
    NONE         --> MOVING        : moveByMm()
    NONE         --> TRAPEZOIDAL   : runProfile()
    NONE         --> SEGMENTED     : runLoadedMove()

    HOMING       --> NONE          : backoff complete → toReady()
    JOG          --> NONE          : stop()
    MOVING       --> NONE          : target reached → toReady()
    TRAPEZOIDAL  --> NONE          : all dips done  → toReady()
    SEGMENTED    --> NONE          : all dips done  → toReady()

    HOMING       --> LIMIT_BACKOFF : unexpected endstop (ISR)
    MOVING       --> LIMIT_BACKOFF : unexpected endstop (ISR)
    TRAPEZOIDAL  --> LIMIT_BACKOFF : unexpected endstop (ISR)
    SEGMENTED    --> LIMIT_BACKOFF : unexpected endstop (ISR)

    LIMIT_BACKOFF --> NONE         : backoff done → toReady()
```

---

## Homing Sequence

The motor must be homed before any positioning command is accepted.
Homing zeros the encoder at the top endstop.

```mermaid
flowchart TD
    A([CMD HOME received]) --> B[set mode = HOMING\nhomingBackoffActive = false]
    B --> C["moveAngle(TRAVEL_MAX_MM upward)\n— commands more than max travel so the\nendstop will always fire before completion"]
    C --> D{Top endstop ISR fires?}
    D -- No --> D
    D -- Yes --> E["stepper.stop(HARD)\nencoder.setHome()  ← zeros position\nhomingBackoffActive = true\nmoveToMm(−HOMING_BACKOFF_MM)"]
    E --> F{isMoveComplete?}
    F -- No --> F
    F -- Yes --> G["mode = NONE\ntoReady()\n← system is now homed at 0 mm"]
```

**Why `moveAngle` instead of `runContinous`?**
`runContinous` forces SpreadCycle regardless of the TPWMTHRS register setting.
`moveAngle` keeps StealthChop active (quiet) during homing.

---

## Trapezoidal Profile

`CMD RUN_PROFILE dip_spd withdraw_spd accel depth_mm dwell_bot_ms dwell_top_ms n_dips`

Each dip cycle follows: Descend → Dwell at bottom → Ascend → Dwell at top → repeat.

```mermaid
flowchart TD
    A([runProfile called]) --> B["profileStartMm = current position\ncurrentDip = 1\nphase = DESCENDING"]
    B --> C["startMoveToMm(profileStart − depth, dipSpeed)"]

    C --> D{isMoveComplete?}
    D -- No --> D

    D -- Yes, phase=DESCENDING --> E["phase = DWELL_BOTTOM\nstartDwell(dwellBottomMs)"]
    E --> F{isDwellComplete?}
    F -- No --> F
    F -- Yes --> G["phase = ASCENDING\nstartMoveToMm(profileStart, withdrawSpeed)"]

    G --> H{isMoveComplete?}
    H -- No --> H
    H -- Yes, phase=ASCENDING --> I{currentDip < nDips?}

    I -- Yes --> J["currentDip++\nphase = DWELL_TOP\nstartDwell(dwellTopMs)"]
    J --> K{isDwellComplete?}
    K -- No --> K
    K -- Yes --> C

    I -- No --> L(["mode = NONE\ntoReady()\nProfile complete"])
```

**Key detail:** `profileStartMm` is recorded once when `runProfile()` is called.
All dip targets and return targets are computed relative to this fixed reference
so encoder drift between dips does not accumulate.

---

## Segmented Move

`CMD BEGIN_SEGMENTED_MOVE` → `CMD MOVE_SEG` × N → `CMD RUN_LOADED_MOVE`

A segmented move allows a user-defined speed profile — e.g. slow entry into the
solution, fast withdrawal.  Each segment is a signed displacement (mm) at a
given speed (mm/s).

```mermaid
flowchart TD
    A([runLoadedMove called]) --> B["segIndex = 0\nphase = DESCENDING\nstartMoveToMm(pos + seg0.dist, seg0.speed)"]

    B --> C{isMoveComplete?}
    C -- No --> C
    C -- Yes --> D["segIndex++"]
    D --> E{segIndex < segCount?}
    E -- Yes --> F["startMoveToMm(pos + segN.dist, segN.speed)"]
    F --> C

    E -- No, phase=DESCENDING --> G["phase = DWELL_BOTTOM\nsegIndex = 0\nstartDwell(dwellBottomMs)"]
    G --> H{isDwellComplete?}
    H -- No --> H
    H -- Yes --> I["phase = ASCENDING\nrun segments 0..N again (opposite directions)"]
    I --> C

    E -- No, phase=ASCENDING --> J{segCurrentDip < segNDips?}
    J -- Yes --> K["segCurrentDip++\nphase = DWELL_TOP\nstartDwell(dwellTopMs)"]
    K --> L{isDwellComplete?}
    L -- No --> L
    L -- Yes --> M["segIndex = 0\nphase = DESCENDING\nrestart descent"]
    M --> C

    J -- No --> N(["mode = NONE\ntoReady()\nDone"])
```

---

## CMD MOVE (Single Relative Move)

`CMD MOVE <dist_mm> [speed_mm_s] [accel_mm_s2]`

A simple point-to-point move used for manual positioning.  Supports pause/resume.

```mermaid
flowchart TD
    A([CMD MOVE received]) --> B["sm.toRunning()\nmoveByMm(dist, speed, accel)"]
    B --> C["mode = MOVING\nstartMoveToMm(currentPos + dist)"]
    C --> D{isMoveComplete?}
    D -- No --> D
    D -- Yes --> E["mode = NONE\ntoReady()\nprint CMD:MOVE:DONE"]
```

**Pause / Resume for CMD MOVE:**

`moveByMm()` saves `speed` and `accel` into `_movingSpeedMms` / `_movingAccelMms2`.
On `resume()`, the MOVING branch re-commands `startMoveToMm(_targetMm, _movingSpeedMms)`,
continuing to the original absolute target from wherever the motor stopped.

---

## isMoveComplete() Logic

This function guards against false completion signals.

```mermaid
flowchart TD
    A([isMoveComplete called]) --> B{_inDwell?}
    B -- Yes --> Z1([return false])

    B -- No --> C{"getMotorState(STANDSTILL) == 1?\n(1 = actively stepping)"}
    C -- Yes --> Z2([return false — motor still moving])

    C -- No --> D["pos      = positionMm()\ntravelMm = |target − moveStart|\nmovedMm  = |pos − moveStart|\nerrorMm  = |pos − target|"]

    D --> E{"travelMm > 0.5 AND\nmovedMm < travelMm × 0.5?"}
    E -- Yes --> Z3([return false — spurious early stop])

    E -- No --> F{"travelMm > 0.5 AND\nerrorMm > 3.0 mm?"}
    F -- Yes --> Z4([return false — stopped far from target])

    F -- No --> G([return true — move complete])
```

**STANDSTILL semantics (inverted from register name):**

| `getMotorState(STANDSTILL)` | Meaning |
|---|---|
| `1` | Motor is **actively stepping** |
| `0` | Motor has **stopped** |

This was confirmed empirically.  The TMC5130 datasheet defines STANDSTILL as a
flag that is set when the motor is stationary, but the UstepperS32 library
wraps it such that the return value is `1` while motion is in progress.

---

## Limit Backoff

When an endstop fires unexpectedly during motion, the firmware:

1. **ISR fires** → hard-stops the motor, sets `_limitTriggered = true`, mode = `LIMIT_BACKOFF`
2. **Next loop()** → `updateLimitBackoff()` reads `_limitTriggered`, prints a message,
   commands a short move away from the endstop, starts a 200 ms guard timer
3. **Subsequent loops** → waits for `isMoveComplete()` or a 5 s safety timeout,
   then transitions to READY

The two-pass approach (ISR sets flag, loop() acts on it) is necessary because
`startMoveToMm()` accesses shared state that is not ISR-safe to modify directly.

---

## Soft Limits

`CMD SET_SOFT_LIMITS <min_mm> <max_mm>`

Every call to `startMoveToMm()` passes the target through `checkSoftLimit()`.
If the target is outside `[_softLimitMinMm, _softLimitMaxMm]`:

1. An error message is printed with the offending target and current limits
2. `estop()` is called — hard-stop, mode = NONE
3. State transitions to `ERROR` with code `SOFT_LIMIT_EXCEEDED`

The defaults from `config.h` span the full physical travel range.

---

## Unit Conversions

The UstepperS32 library uses **degrees** for all positioning, while the rest of
the firmware uses **millimetres**.

```
degrees = mm × (360 / LEADSCREW_MM_PER_REV)
mm      = degrees × (LEADSCREW_MM_PER_REV / 360)
```

`LEADSCREW_MM_PER_REV` is defined in `config.h` and can be calibrated using
`DIAG CAL` — see [DIAGNOSTICS.md](DIAGNOSTICS.md).
