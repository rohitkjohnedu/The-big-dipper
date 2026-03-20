# Dip Coater Firmware — Architecture Overview

## Purpose

This firmware runs on a **uStepper S32** (STM32-based) board and controls a vertical
dip coater: a motor-driven stage that dips a substrate into a solution at programmable
speeds, dwells, and depths.

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | uStepper S32 (STM32F103) |
| Stepper driver | TMC5130 (on-board, via UstepperS32 library) |
| Motor mode | StealthChop below ~3 mm/s, SpreadCycle above |
| Endstops | Active-LOW, INPUT_PULLUP; top = home, bottom = travel limit |
| Communication | USB-Serial, 115200 baud, UTF-8, `\n` or `\r\n` terminated |

### Coordinate system

```
Home position (top endstop) = 0 mm
Positive mm = upward   (toward home)
Negative mm = downward (into solution)
```

---

## Module Map

```mermaid
graph TD
    INO["dip_coater_firmware.ino\n(setup / loop)"]
    CP["CommandParser\n(serial I/O + dispatch)"]
    MC["MotionController\n(stepper motion)"]
    SM["StateMachine\n(state + phase tracking)"]
    DI["Diagnostics\n(self-tests)"]
    LIB["UstepperS32\n(TMC5130 driver)"]
    HW["Hardware\n(motor + endstops)"]

    INO -->|"begin() / update()"| CP
    INO -->|"begin() / update()"| MC
    INO -->|"begin() / update()"| DI
    CP  -->|"executeHome / moveByMm / jog …"| MC
    CP  -->|"startMotorTest / startMoveTest …"| DI
    CP  -->|"toRunning / toPaused …"| SM
    MC  -->|"toReady / toError …"| SM
    DI  -->|"moveByMm / jog / stop"| MC
    DI  -->|"getState"| SM
    MC  -->|"setup / moveToAngle …"| LIB
    LIB -->|"step pulses + current"| HW
    HW  -->|"endstop ISR"| MC
```

---

## Top-Level System States

```mermaid
stateDiagram-v2
    [*] --> IDLE : power on

    IDLE    --> HOMING  : CMD HOME
    READY   --> HOMING  : CMD HOME (re-home)
    ERROR   --> HOMING  : CMD HOME (recovery)

    HOMING  --> READY   : endstop hit + backoff complete

    READY   --> RUNNING : CMD RUN_PROFILE\nCMD BEGIN_SEGMENTED_MOVE + RUN_LOADED_MOVE\nCMD MOVE

    RUNNING --> PAUSED  : CMD PAUSE
    PAUSED  --> RUNNING : CMD RESUME

    RUNNING --> READY   : profile complete\nCMD STOP
    PAUSED  --> READY   : CMD STOP

    RUNNING --> ERROR   : unexpected endstop\nsoft limit exceeded
    PAUSED  --> ERROR   : (same faults)
    HOMING  --> ERROR   : (same faults)

    note right of IDLE
        Only CMD HOME accepted.
        Motor not yet zeroed.
    end note

    note right of ERROR
        Requires CMD HOME
        or hardware reset.
    end note
```

---

## Run Phase (sub-state while RUNNING)

While in the `RUNNING` state the firmware tracks a finer **RunPhase** that
describes where in the dip cycle the system currently is.

```mermaid
stateDiagram-v2
    [*]           --> DESCENDING  : profile start
    DESCENDING    --> DWELL_BOTTOM: target depth reached
    DWELL_BOTTOM  --> ASCENDING   : dwell timer expired
    ASCENDING     --> DWELL_TOP   : home position reached (more dips remain)
    ASCENDING     --> [*]         : home position reached (last dip — READY)
    DWELL_TOP     --> DESCENDING  : dwell timer expired
```

> **Note:** For `CMD MOVE` (single-move mode) the phase stays at `NONE`
> throughout — phases only apply to trapezoidal and segmented profiles.

---

## main loop() call order

```mermaid
sequenceDiagram
    participant L  as loop()
    participant CP as CommandParser
    participant MC as MotionController
    participant DI as Diagnostics

    loop every ~1 ms
        L->>CP: update()
        Note over CP: read Serial, dispatch commands
        L->>MC: update()
        Note over MC: advance active motion mode
        L->>DI: update()
        Note over DI: advance active diagnostic test
    end
```

All three modules are **non-blocking** — they never call `delay()`.  The loop
runs at full MCU speed (≈ 1 MHz effective) so motion timing is smooth.

---

## File List

| File | Responsibility |
|---|---|
| `config.h` | All tunable constants (pin numbers, speeds, limits, motor geometry) |
| `state_machine.h/.cpp` | System state and run-phase enumerations and transitions |
| `motion_controller.h/.cpp` | Stepper motion — homing, profiles, jog, single moves |
| `diagnostics.h/.cpp` | Hardware self-tests triggered by DIAG commands |
| `command_parser.h/.cpp` | Serial line accumulation, command dispatch, response formatting |
| `trace.h` | Lightweight serial trace macros (zero cost when `TRACE` is not defined) |
| `dip_coater_firmware.ino` | Arduino sketch entry point — `setup()` and `loop()` |

---

## Detailed Documentation

- [State Machine](STATE_MACHINE.md)
- [Motion Controller](MOTION_CONTROLLER.md)
- [Command Protocol](COMMAND_PROTOCOL.md)
- [Diagnostics Module](DIAGNOSTICS.md)
