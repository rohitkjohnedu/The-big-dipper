Dip Coater Control System — Full Architecture Brief
====================================================

Last updated: 2026-03-24

## Project Overview

A dip-coating rig controller with two components: Arduino firmware for the motion controller,
and a Python desktop UI. The primary engineering priority is reproducible velocity profiles —
speed and acceleration consistency matters more than absolute position accuracy.

---

## Repository Structure

```
dipping_sequence/
├── CLAUDE.md                        ← Claude Code session guide
├── claude_design.md                 ← you are here (original design brief, kept current)
├── claude_context.md                ← session-continuity reference
├── readme.md
├── arduino/
│   └── dip_coater_firmware/
│       ├── dip_coater_firmware.ino  ✅
│       ├── command_parser.h/.cpp    ✅
│       ├── state_machine.h/.cpp     ✅
│       ├── motion_controller.h/.cpp ✅
│       ├── telemetry.h/.cpp         ✅
│       ├── diagnostics.h/.cpp       ✅
│       └── config.h                 ✅
└── python/
    ├── pyproject.toml
    ├── uv.lock
    ├── .python-version
    ├── main.py                      (Step 26 — not yet built)
    ├── mock_demo.py                 ✅
    ├── widget_demo.py               ✅
    ├── hw_test.py                   ✅
    ├── core/
    │   ├── serial_manager.py        ✅ (includes raw_queue)
    │   ├── command_interface.py     ✅
    │   ├── telemetry_parser.py      ✅
    │   ├── data_recorder.py         ✅
    │   └── profile.py               ✅
    ├── motion/
    │   ├── velocity_profile.py      ✅ abstract base class + MoveSegment
    │   ├── trapezoidal_profile.py   ✅
    │   ├── parabolic_profile.py     ✅ hardware-tested 2026-03-21
    │   └── spline_profile.py        ✅ stub — NotImplementedError
    ├── ui/
    │   ├── widgets/
    │   │   ├── estop_button.py      ✅
    │   │   ├── live_plot.py         ✅
    │   │   └── status_bar.py        ✅
    │   └── tabs/
    │       ├── control_tab.py       ✅
    │       ├── serial_monitor_tab.py ✅
    │       ├── telemetry_tab.py     ⬜ Step 22
    │       ├── profile_manager_tab.py ⬜ Step 23
    │       └── profile_editor_tab.py  ⬜ Step 24 (stub)
    ├── tests/
    │   ├── conftest.py              ✅
    │   ├── mock_arduino.py          ✅ 51 tests
    │   ├── test_mock_arduino.py     ✅
    │   ├── core/                    ✅ ~105 tests
    │   ├── motion/                  ✅ ~45 tests
    │   ├── ui/                      ✅ 82 tests (test_widgets + test_tabs)
    │   └── hardware/                ✅ hardware-in-the-loop
    └── profiles/
        └── default.json
```

---

## Hardware

| Item | Value |
|------|-------|
| Microcontroller | uStepper S32 |
| Motor | NEMA 23 stepper |
| Travel | ~900mm vertical axis |
| End switches | Two (top and bottom), active LOW, internal pull-up, interrupt pins |
| Library | uStepperS32.h — closed-loop PID, encoder feedback |
| Port | COM4 (Windows) |
| Baud | 115200 |

The firmware sits on top of uStepperS32.h as a **command interpreter and telemetry forwarder**.
Do not reimplement motion control — use the library.

---

## Units Strategy

| Layer | Unit |
|-------|------|
| Arduino internal | steps (integer arithmetic) |
| Serial wire (both directions) | **mm, mm/s, mm/s²** — no exceptions |
| Python | mm throughout — no conversion needed |

Conversion lives **only on the Arduino** in `config.h` as `STEPS_PER_MM`.
`unit_converter.py` does not exist — it was excluded because it would be a drift-bug source.

---

## config.h — Written and Hardware-Verified

```cpp
#define MOTOR_STEPS_PER_REV       200
#define MICROSTEPS                32
#define LEADSCREW_MM_PER_REV      8.0f
#define STEPS_PER_MM              800.0f   // (200 * 32) / 8.0

#define TRAVEL_MAX_MM             900.0f
#define SOFT_LIMIT_MIN_MM         10.0f
#define SOFT_LIMIT_MAX_MM         890.0f
#define HOMING_SPEED_MM_S         20.0f
#define HOMING_BACKOFF_MM         5.0f

#define PIN_ENDSTOP_BOTTOM        2
#define PIN_ENDSTOP_TOP           3
#define PIN_STATUS_LED            13

#define SERIAL_BAUD_RATE          115200
#define SERIAL_BUFFER_SIZE        128
#define DEFAULT_TELEM_RATE_HZ     1
#define MAX_TELEM_RATE_HZ         50

#define DEFAULT_DIP_SPEED_MM_S        10.0f
#define DEFAULT_WITHDRAW_SPEED_MM_S   10.0f
#define DEFAULT_ACCEL_MM_S2           50.0f
#define DEFAULT_DIP_DEPTH_MM          100.0f
#define DEFAULT_DWELL_BOTTOM_MS       1000
#define DEFAULT_DWELL_TOP_MS          500
#define DEFAULT_N_DIPS                1

#define MOVE_SEG_BUFFER_SIZE      64
```

---

## State Machine — Written

### States
```
IDLE    → powered on, not homed. Only HOME command accepted.
HOMING  → driving to bottom end switch to zero encoder.
READY   → homed, stationary. All commands accepted.
RUNNING → executing dip profile.
PAUSED  → mid-profile pause, position held. RESUME or STOP accepted.
ERROR   → fault. Requires HOME or reset to recover.
```

### Valid transitions
```
IDLE    → HOMING
HOMING  → READY | ERROR
READY   → RUNNING | HOMING | ERROR
RUNNING → PAUSED | READY (profile complete) | ERROR
PAUSED  → RUNNING (resume) | READY (stop) | ERROR
ERROR   → HOMING (recovery) | IDLE (soft reset)
```

### Run phases (sub-state during RUNNING / PAUSED)
```
NONE | DESCENDING | DWELL_BOTTOM | ASCENDING | DWELL_TOP
```

### Error codes
```
NONE
ENDSTOP_TRIGGERED_UNEXPECTEDLY
SOFT_LIMIT_EXCEEDED
COMMAND_INVALID_STATE
COMMAND_PARSE_ERROR
PROFILE_INVALID
SEG_BUFFER_OVERFLOW
```

---

## Serial Protocol

All messages newline-terminated (`\n`). All values on the wire in mm, mm/s, mm/s².

### Commands (Python → Arduino)

**Lifecycle**
```
CMD HOME
CMD GET_STATE
CMD SET_TELEM_RATE <hz>              # 0 disables, max 50
CMD SET_SOFT_LIMITS <min_mm> <max_mm>
CMD DEBUG <ON|OFF>
```

**Motion — stop/pause/resume**
```
CMD STOP                             # controlled decelerated stop
CMD ESTOP                            # immediate hard stop, no ramp
CMD PAUSE                            # decelerated stop, saves profile state
CMD RESUME                           # re-accelerates, continues saved state
CMD JOG <UP|DOWN> <speed_mm_s>       # manual movement, READY state only
```

**Motion — trapezoidal profile (single command)**
```
CMD RUN_PROFILE <dip_speed_mm_s> <withdraw_speed_mm_s> <accel_mm_s2> <depth_mm> <dwell_bottom_ms> <dwell_top_ms> <n_dips>
```
Arduino executes the full dip cycle autonomously. Used for all trapezoidal profiles.

**Motion — segmented profile (streaming)**
```
CMD BEGIN_SEGMENTED_MOVE <n_segments>
CMD MOVE_SEG  <distance_mm> <speed_mm_s> [accel_mm_s2]   # no per-segment ACK
CMD DWELL_SEG <duration_ms>                               # no per-segment ACK
# → Arduino sends ACK PROFILE_READY after the last segment
CMD RUN_LOADED_MOVE
```
Python pre-computes the full velocity profile, samples it into short segments, and streams
the full list before triggering execution. Same principle as 3D printer slicers.

### Responses (Arduino → Python)
```
ACK <COMMAND>
ACK PROFILE_READY
ERR <COMMAND> <reason>
ERR SEG_BUFFER_OVERFLOW
STATE <state> <phase>
```

### Telemetry (Arduino → Python, at configured rate)
```
TELEM,<millis>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_actual_mm_s2>,<state>,<phase>
```
Both commanded and actual velocity are always broadcast — the S32 closed-loop PID means they
may differ during ramps. Logging both verifies profile fidelity post-run.

---

## Arduino Firmware Architecture

Four modules on top of uStepperS32.h. The firmware is a command interpreter and telemetry
forwarder — motion control is the library's responsibility.

### command_parser.h/.cpp
- Reads serial buffer character by character in `loop()`
- On `\n`, tokenises and dispatches to the appropriate handler
- Validates command against current state machine state
- Returns ACK or ERR on every command — no silent failures
- Handles multi-line segmented move handshake: after `BEGIN_SEGMENTED_MOVE`, collects
  `MOVE_SEG` lines until count reached, then sends `ACK PROFILE_READY`

### state_machine.h/.cpp
- Owns `SystemState`, `RunPhase`, `ErrorCode`
- Exposes transition methods and `canRun()`, `canPause()`, `canResume()`, `canJog()` guards

### motion_controller.h/.cpp
- Owns the uStepperS32 instance
- Executes HOME, RUN_PROFILE, segmented move, JOG, STOP, ESTOP, PAUSE, RESUME
- `RUN_PROFILE`: runs full dip cycle autonomously, non-blocking, millis()-based timing
- `RUN_LOADED_MOVE`: executes buffered segments sequentially, continuous with no gaps
- Calls `state_machine.setPhase()` as it transitions between run phases
- End switch ISRs call `motion_controller.onEndstopTriggered()` → `state_machine.toError()`
- PAUSE: decelerates, saves remaining segments and current phase
- RESUME: re-accelerates, continues from saved segment index

### telemetry.h/.cpp
- Owns telem rate and last-broadcast timestamp
- Called every `loop()` — checks interval, broadcasts if elapsed
- Reads position and velocity from uStepperS32 encoder, converts to mm before sending
- Formats and prints TELEM CSV line

### dip_coater_firmware.ino
- `setup()`: initialise serial, uStepperS32, pins, endstop interrupts, state machine
- `loop()`: call command_parser update, motion_controller update, telemetry update — nothing else
- **Critical rule: no `delay()` anywhere. All timing via `millis()`.**

---

## Python Architecture

### Threading model

```
Serial thread   → reads incoming lines, routes to telem_queue / response_queue / raw_queue
UI/main thread  → PyQt6 event loop, QTimer polls queues via signals/slots, never blocks
Worker thread   → QThread (_RunWorker in ControlTab) runs CommandInterface.run() during
                  segmented profile streaming (blocks while sending segments)
```

### core/serial_manager.py
- Owns the `serial.Serial` connection
- Background read thread routes lines to three queues:
  - `telem_queue` — parsed `TelemetryFrame` objects
  - `response_queue` — raw ACK/ERR/STATE strings for `CommandInterface`
  - `raw_queue` — copy of every TX/RX line for `SerialMonitorTab`
- `send_command(cmd)` — thread-safe, write-lock protected

### core/command_interface.py
- Builds and sends command strings from typed Python arguments
- Inspects `DipProfile.velocity_profile_type` to choose wire path:
  - `"trapezoidal"` → single `CMD RUN_PROFILE` line
  - `"parabolic"` or `"segmented"` → `BEGIN_SEGMENTED_MOVE` streaming path
  - `"spline"` → raises `NotImplementedError`
- Awaits ACK/ERR with timeout on every command
- ESTOP is sent immediately without ACK wait

### core/telemetry_parser.py
- Parses `TELEM,...` strings into `TelemetryFrame` dataclass
- Malformed lines logged as warning, never crash

### core/data_recorder.py
- Maintains numpy arrays of all telemetry fields for the current run
- Auto-saves CSV to `logs/run_<name>_<timestamp>.csv` on run complete

### core/profile.py
```python
@dataclass
class DipProfile:
    name:                   str
    dip_speed_mm_s:         float
    withdraw_speed_mm_s:    float
    accel_mm_s2:            float
    dip_depth_mm:           float
    dwell_bottom_ms:        int
    dwell_top_ms:           int
    n_dips:                 int
    notes:                  str  = ""
    created_at:             str  = ""
    velocity_profile_type:  str  = "trapezoidal"
    velocity_profile_data:  dict = field(default_factory=dict)
```

---

## Motion / Velocity Profile Architecture

Python does the maths; Arduino executes simple move primitives.

### Abstract base (velocity_profile.py)
```python
class VelocityProfile(ABC):
    def get_speed_at(self, t: float) -> float: ...
    def total_duration(self) -> float: ...
    def to_segments(self, segment_length_mm: float = 1.0) -> list[MoveSegment]: ...
    def to_dict(self) -> dict: ...
    @classmethod
    def from_dict(cls, data: dict) -> VelocityProfile: ...
```

### Implemented profiles

| Class | Type string | Wire path | Status |
|-------|-------------|-----------|--------|
| `TrapezoidalProfile` | `"trapezoidal"` | Single `CMD RUN_PROFILE` | ✅ Hardware-tested |
| `ParabolicProfile` | `"parabolic"` | Segmented streaming | ✅ Hardware-tested 2026-03-21 |
| `SplineProfile` | `"spline"` | Segmented streaming | ⬜ Stub (Step 27) |

`to_segments()` must check `segment_count ≤ 64` before touching serial.

---

## UI Architecture

```
MainWindow (QMainWindow)
├── EstopButton                ✅ — always visible, calls ci.estop() immediately
├── QTabWidget
│   ├── ControlTab             ✅ — connection, home, jog, run/pause/stop, profile picker
│   ├── SerialMonitorTab       ✅ — raw TX/RX log, multi-line command input, autocomplete
│   ├── TelemetryTab           ⬜ — LivePlot + DataRecorder controls
│   ├── ProfileManagerTab      ⬜ — CRUD for JSON profiles
│   └── ProfileEditorTab       ⬜ — stub (spline waypoint editor)
└── StatusBar                  ✅ — port, Arduino state, phase, elapsed time
```

### ControlTab (control_tab.py)
- Three `QGroupBox` sections: Connection, Manual Control, Run Profile
- `_RunWorker(QThread)` runs `ci.run(profile)` off the UI thread
- Button states strictly enforce the Arduino state machine
- Signals: `connect_requested(port, baud)`, `disconnect_requested()`
- Public API: `set_command_interface(ci)`, `update_state(state)`, `set_profiles(profiles)`

### SerialMonitorTab (serial_monitor_tab.py)
- `QPlainTextEdit` log pane — read-only, colour-coded, max 2000 lines
  - TX=amber, ACK=green, ERR=red, TELEM=grey, other=white
- `_CommandInput(QPlainTextEdit)` subclass:
  - Ctrl+Enter sends all lines sequentially
  - Tab accepts autocomplete (QCompleter, contains-match, case-insensitive)
  - Up/Down cycles through sent-batch history
  - Multi-line support: write N commands separated by newlines, send all at once
- History sidebar (QListWidget): all sent batches, click to reload
- Quick-command dropdown with common firmware commands
- Auto-scroll toggle and Clear button
- Polls `manager.raw_queue` at 20 Hz

### EstopButton, LivePlot, StatusBar (widgets/)
- `EstopButton` — large red button, always enabled, calls `ci.estop()` on click
- `LivePlot` — three stacked pyqtgraph subplots (position, velocity, acceleration)
- `StatusBar` — port indicator, state badge (colour-coded), phase, elapsed time

---

## MockArduino — Testing Architecture

`MockArduino` is a full Arduino simulator in Python. It is **not** a replacement for hardware
testing — it is a separate tool for a separate purpose.

### Why it exists

Physical hardware requires the rig to be present, powered, and exclusively connected. It cannot
be used in development away from the bench, cannot inject fault conditions, and runs at real
clock speed (8–15 s per profile). `MockArduino` runs at configurable speed (default 5×),
starts clean every test, and can simulate endstop faults, buffer overflows, and ERR responses.

### Two-tier strategy

```
Tier 1 — MockArduino — all 397 non-hardware tests
Tier 2 — Real hardware COM4 — uv run pytest -m hardware --hw-port COM4
```

### Public interface (identical to SerialManager)
```python
telem_queue:    queue.Queue[TelemetryFrame]
response_queue: queue.Queue[str]
raw_queue:      queue.Queue[str]
send_command(cmd: str) → None
start() / stop()
is_connected: bool
```

---

## Testing Strategy

### Test files and counts (non-hardware, as of 2026-03-24)

| File | Count | What it tests |
|------|-------|---------------|
| `test_telemetry_parser.py` | ~30 | Valid/malformed TELEM lines, boundary values |
| `test_profile.py` | ~25 | Save/load round-trip, validation |
| `test_command_interface.py` | ~30 | Command strings, ACK/ERR, timeout, dispatch |
| `test_data_recorder.py` | ~20 | CSV columns, auto-save |
| `test_trapezoidal_profile.py` | ~25 | Profile shape, duration, segment count |
| `test_parabolic_profile.py` | ~20 | Bell-curve shape, segment count |
| `test_mock_arduino.py` | 51 | MockArduino state machine, all commands |
| `test_widgets.py` | 25 | EstopButton, LivePlot, StatusBar |
| `test_tabs.py` | 57 | ControlTab (33), SerialMonitorTab (24) |
| **Total** | **397** | |

### Running tests
```bash
cd python
uv run pytest                          # all 397
uv run pytest tests/ui/ -v             # UI tests only
uv run pytest -m hardware --hw-port COM4 -v   # hardware-in-the-loop
```

---

## Build Order

```
Step  File                                   Status
----  ----                                   ------
 1    arduino/config.h                       ✅ done
 2    arduino/state_machine.h/.cpp           ✅ done
 3    arduino/command_parser.h/.cpp          ✅ done
 4    arduino/motion_controller.h/.cpp       ✅ done
 5    arduino/telemetry.h/.cpp               ✅ done
 6    arduino/dip_coater_firmware.ino        ✅ done
 7    core/telemetry_parser.py               ✅ done + tests
 8    core/profile.py                        ✅ done + tests
 9    core/serial_manager.py                 ✅ done (+ raw_queue)
10    core/command_interface.py              ✅ done + tests
11    core/data_recorder.py                  ✅ done + tests
12    motion/velocity_profile.py             ✅ done (abstract base + MoveSegment)
13    motion/trapezoidal_profile.py          ✅ done + tests
14    motion/parabolic_profile.py            ✅ done + tests
15    motion/spline_profile.py               ✅ done (stub)
16    tests/mock_arduino.py                  ✅ done — 51 tests
17    ui/widgets/estop_button.py             ✅ done + tests
18    ui/widgets/live_plot.py                ✅ done + tests
19    ui/widgets/status_bar.py               ✅ done + tests
20    ui/tabs/control_tab.py                 ✅ done + 33 tests
21    ui/tabs/serial_monitor_tab.py          ✅ done + 24 tests
22    ui/tabs/telemetry_tab.py               ⬜ next
23    ui/tabs/profile_manager_tab.py         ⬜
24    ui/tabs/profile_editor_tab.py          ⬜ stub
25    ui/main_window.py                      ⬜
26    main.py                                ⬜
27    motion/spline_profile.py (implement)   ⬜ after UI complete
```

---

## Coding Rules

1. **No `delay()` anywhere in the Arduino firmware.** All timing via `millis()`.
2. **Serial thread never touches PyQt6 widgets.** Only signals/slots cross thread boundaries.
3. **Unit conversion happens only on the Arduino.** Wire always speaks mm. Python never converts.
4. **Every command gets an ACK or ERR.** No silent failures.
5. **ESTOP bypasses all queues** — sent immediately and directly.
6. **`command_interface.run()` inspects `velocity_profile_type`** and dispatches automatically.
7. **`to_segments()` must check segment count against `MOVE_SEG_BUFFER_SIZE` (64)** and raise before any serial communication.
8. **`profile_editor_tab.py` and `spline_profile.py` are stubs** — correct signatures, raise `NotImplementedError`, must not be skipped.
9. **All tests run against `MockArduino`** — no test requires hardware except `tests/hardware/`.
10. **`logs/` and user `profiles/` are never committed.** `profiles/default.json` is the only committed profile.
11. **`uv.lock` is always committed.**
12. **One responsibility per module.**

---

## Package Management

Tool: `uv`

```bash
uv sync                  # install all dependencies
uv sync --extra dev      # include test dependencies
uv run pytest            # run tests
uv run python widget_demo.py   # widget preview (no hardware)
uv run python main.py          # full app (Step 26 — not yet built)
```

Dependencies: `pyserial`, `PyQt6`, `pyqtgraph`, `numpy`, `scipy`
Dev: `pytest`, `pytest-qt`
