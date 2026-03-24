# Dip Coater Control System — CLAUDE.md

Project context and coding rules for Claude Code.
Read this file at the start of every session before touching any code.

---

## Project Overview

A dip-coating rig controller with two components:

- **Arduino firmware** — motion controller running on a uStepper S32 (NEMA 23 stepper, ~900 mm vertical axis)
- **Python desktop UI** — PyQt6 application for operating the rig, visualising telemetry, and managing coating profiles

The primary engineering priority is **reproducible velocity profiles** — speed and acceleration consistency matters more than absolute position accuracy.

All Python work lives in `python/`. Run every command from that directory:

```bash
cd python
uv run pytest                      # all unit tests (no hardware needed)
uv run python main.py              # launch full app (Step 26 — not yet built)
uv run python widget_demo.py       # live widget preview (EstopButton + LivePlot + StatusBar)
uv run python mock_demo.py         # raw MockArduino console demo
```

---

## Repository Structure

```
dipping_sequence/
├── CLAUDE.md                        ← you are here
├── claude_design.md                 ← original design brief (reference)
├── claude_context.md                ← comprehensive session-continuity reference
├── readme.md
├── arduino/
│   └── dip_coater_firmware/
│       ├── dip_coater_firmware.ino  ✅ written
│       ├── command_parser.h/.cpp    ✅ written
│       ├── state_machine.h/.cpp     ✅ written
│       ├── motion_controller.h/.cpp ✅ written
│       ├── telemetry.h/.cpp         ✅ written
│       ├── diagnostics.h/.cpp       ✅ written
│       └── config.h                 ✅ written
└── python/
    ├── pyproject.toml               (PyQt6, pyqtgraph, numpy, scipy, pyserial)
    ├── main.py                      (stub — Step 26)
    ├── mock_demo.py                 ✅ console demo of MockArduino
    ├── widget_demo.py               ✅ visual demo of UI widgets
    ├── hw_test.py                   ✅ ad-hoc hardware test script
    ├── logs/                        (CSV telemetry recordings — not committed)
    ├── core/
    │   ├── serial_manager.py        ✅ (includes raw_queue)
    │   ├── command_interface.py     ✅
    │   ├── telemetry_parser.py      ✅
    │   ├── data_recorder.py         ✅
    │   └── profile.py               ✅
    ├── motion/
    │   ├── velocity_profile.py      ✅ abstract base + MoveSegment
    │   ├── trapezoidal_profile.py   ✅
    │   ├── parabolic_profile.py     ✅
    │   └── spline_profile.py        ✅ stub — all methods raise NotImplementedError
    ├── ui/
    │   ├── widgets/
    │   │   ├── estop_button.py      ✅ Step 17
    │   │   ├── live_plot.py         ✅ Step 18
    │   │   └── status_bar.py        ✅ Step 19
    │   └── tabs/
    │       ├── control_tab.py       ✅ Step 20
    │       ├── serial_monitor_tab.py ✅ Step 21
    │       ├── telemetry_tab.py     ⬜ Step 22
    │       ├── profile_manager_tab.py ⬜ Step 23
    │       └── profile_editor_tab.py  ⬜ Step 24 (stub)
    └── tests/
        ├── conftest.py              ✅ hw fixtures + pytest_configure hook
        ├── mock_arduino.py          ✅ Step 16 — full Arduino simulator (includes raw_queue)
        ├── test_mock_arduino.py     ✅ 51 tests
        ├── core/
        │   ├── test_command_interface.py  ✅
        │   ├── test_telemetry_parser.py   ✅
        │   ├── test_profile.py            ✅
        │   └── test_data_recorder.py      ✅
        ├── motion/
        │   ├── test_trapezoidal_profile.py ✅
        │   └── test_parabolic_profile.py   ✅
        ├── ui/
        │   ├── test_widgets.py             ✅ 25 tests (pytest-qt)
        │   └── test_tabs.py                ✅ 57 tests (pytest-qt)
        └── hardware/
            └── test_hardware.py            ✅ hardware-in-the-loop tests
```

---

## Build Order — Current Status

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
14    motion/parabolic_profile.py            ✅ done + tests (segmented path)
15    motion/spline_profile.py               ✅ done (stub — NotImplementedError)
16    tests/mock_arduino.py                  ✅ done — 51 tests passing (+ raw_queue)
17    ui/widgets/estop_button.py             ✅ done + tests
18    ui/widgets/live_plot.py                ✅ done + tests
19    ui/widgets/status_bar.py               ✅ done + tests
20    ui/tabs/control_tab.py                 ✅ done + tests (33 tests)
21    ui/tabs/serial_monitor_tab.py          ✅ done + tests (24 tests)
22    ui/tabs/telemetry_tab.py               ⬜ next
23    ui/tabs/profile_manager_tab.py         ⬜
24    ui/tabs/profile_editor_tab.py          ⬜ stub
25    ui/main_window.py                      ⬜
26    main.py                                ⬜
(27)  motion/spline_profile.py (implement)   ⬜ after UI complete
```

**Total non-hardware tests passing: 397**

---

## Hardware

| Item | Value |
|------|-------|
| MCU | uStepper S32 |
| Motor | NEMA 23 stepper |
| Travel | ~900 mm vertical |
| Endstops | Two (top + bottom), active LOW, internal pull-up, interrupt pins |
| Library | uStepperS32.h — closed-loop PID, encoder feedback |
| Port | COM4 (Windows) |
| Baud | 115200 |

**Do not reimplement motion control** — the uStepperS32 library handles closed-loop PID natively. The firmware is a command interpreter and telemetry forwarder on top of that library.

### config.h constants (verified on hardware)

```cpp
#define STEPS_PER_MM              800.0f   // (200 * 32) / 8.0
#define TRAVEL_MAX_MM             900.0f
#define SOFT_LIMIT_MIN_MM         10.0f
#define SOFT_LIMIT_MAX_MM         890.0f
#define HOMING_SPEED_MM_S         20.0f
#define SERIAL_BAUD_RATE          115200
#define MOVE_SEG_BUFFER_SIZE      64
#define MAX_TELEM_RATE_HZ         50
```

---

## Units Strategy

| Layer | Unit |
|-------|------|
| Arduino internal | steps (integer) |
| Serial wire (both directions) | **mm, mm/s, mm/s²** always |
| Python | mm throughout — no conversion |

Unit conversion lives **only on the Arduino** in `config.h` as `STEPS_PER_MM`.
`unit_converter.py` does not exist and must never be created — it would be a drift-bug source.

---

## State Machine

```
IDLE    → powered on, not homed. Only HOME accepted.
HOMING  → driving to bottom endstop to zero encoder.
READY   → homed, stationary. All commands accepted.
RUNNING → executing dip profile.
PAUSED  → mid-profile pause, position held. RESUME or STOP accepted.
ERROR   → fault. Requires HOME or reset to recover.
```

Valid transitions:
```
IDLE    → HOMING
HOMING  → READY | ERROR
READY   → RUNNING | HOMING | ERROR
RUNNING → PAUSED | READY (profile complete) | ERROR
PAUSED  → RUNNING (resume) | READY (stop) | ERROR
ERROR   → HOMING (recovery) | IDLE (soft reset)
```

Run phases (sub-state during RUNNING/PAUSED):
```
NONE | DESCENDING | DWELL_BOTTOM | ASCENDING | DWELL_TOP
```

---

## Serial Protocol

All messages newline-terminated (`\n`). Values always in mm/mm·s/mm·s².

### Commands (Python → Arduino)

```
CMD HOME
CMD STOP
CMD ESTOP
CMD PAUSE
CMD RESUME
CMD GET_STATE                               → STATE <state> <phase>
CMD JOG <UP|DOWN> <speed_mm_s> [accel]
CMD SET_TELEM_RATE <hz>
CMD SET_SOFT_LIMITS <min_mm> <max_mm>

# Trapezoidal profile — single command:
CMD RUN_PROFILE <dip_spd> <wdraw_spd> <accel> <depth> <dwell_bot_ms> <dwell_top_ms> <n_dips>

# Segmented profile — streaming protocol:
CMD BEGIN_SEGMENTED_MOVE <n_segs>
CMD MOVE_SEG  <dist_mm> <speed_mm_s> [accel_mm_s2]   # no per-segment ACK
CMD DWELL_SEG <duration_ms>                           # no per-segment ACK
# → Arduino sends ACK PROFILE_READY after the last segment
CMD RUN_LOADED_MOVE
```

### Responses (Arduino → Python)

```
ACK <COMMAND>
ACK PROFILE_READY
ERR <COMMAND> <reason>
STATE <state> <phase>
TELEM,<millis>,<pos_mm>,<vel_actual>,<vel_commanded>,<accel>,<state>,<phase>
```

Intermediate `MOVE_SEG` / `DWELL_SEG` lines produce **no per-segment ACK** — only the final segment triggers `ACK PROFILE_READY`.

---

## MockArduino — Why It Exists

`MockArduino` is a **Python-only Arduino simulator** used by all non-hardware unit tests.
It is NOT a replacement for physical testing — it is a different tool for a different purpose.

### Two-tier testing strategy

```
Tier 1 — MockArduino (no hardware, <30 s, always available)
  → All 397 unit tests run here
  → Covers: protocol correctness, state-machine enforcement, error handling,
            UI widget behaviour, profile serialisation, velocity mathematics

Tier 2 — Real hardware on COM4 (requires rig powered on bench)
  → Covers: actual motion, endstop response, velocity profile fidelity,
            encoder feedback, UART timing
```

### Why hardware-only testing is insufficient

- Tests require the rig to be physically present and powered — cannot run in development
- COM4 is exclusive — only one process holds it at a time (not compatible with Arduino IDE open)
- Real runs take 8–15 s each; MockArduino runs at configurable speed multiplier (5× default)
- Cannot inject fault conditions (unexpected endstop, buffer overflow) on real hardware
- Firmware bugs make hardware tests ambiguous — you cannot isolate whether Python or Arduino is wrong

### MockArduino interface contract

`MockArduino` exposes the **identical public API** as `SerialManager`, so
`CommandInterface(mock)` is byte-for-byte identical to `CommandInterface(serial_manager)`:

```python
telem_queue:    queue.Queue[TelemetryFrame]
response_queue: queue.Queue[str]
raw_queue:      queue.Queue[str]        # TX/RX log for serial monitor tab
send_command(cmd: str) → None
start() / stop()
is_connected: bool
```

`send_command()` is synchronous in MockArduino — it dispatches the command, applies state
transitions, and puts the ACK/ERR on `response_queue` immediately (no physical UART delay).

---

## Key Technical Decisions (made during development)

### Parabolic profile
Bell-curve `v(x) = v_peak × 4(x/d)(1−x/d)` — averages (2/3)×peak, smooth entry/exit.
Implemented as segmented profile (streams via `BEGIN_SEGMENTED_MOVE` path).
Lives in `motion/parabolic_profile.py`. Hardware-tested on COM4 (2026-03-21).

### raw_queue — serial monitor data feed
Both `SerialManager` and `MockArduino` have a third queue:
```python
raw_queue: queue.Queue[str]   # maxsize=2000
```
- TX lines: `"TX CMD HOME"`
- RX lines: `"RX ACK HOME"`, `"RX TELEM,12453,..."`

`SerialMonitorTab` polls `raw_queue` at 20 Hz. The existing `telem_queue` and
`response_queue` consumers are completely unaffected — `raw_queue` is additive.

### Serial monitor tab (Step 21)
Added as a dedicated tab between ControlTab and TelemetryTab. Features:
- Colour-coded log: TX=amber, ACK=green, ERR=red, TELEM=grey
- `_CommandInput` subclass — Ctrl+Enter send, Tab autocomplete, Up/Down history
- Multi-line input: write several commands separated by newlines, send all at once
- History sidebar: all sent batches, click to reload
- Quick-command dropdown with common firmware commands

### Inter-segment delay
`INTER_SEGMENT_DELAY_S = 0.010` (10 ms) in `command_interface.py`.
Prevents the Arduino's 64–256 byte UART RX buffer from overflowing when streaming many segments back-to-back.

### SplineProfile stub
All methods raise `NotImplementedError`. The class exists so:
- `from motion.spline_profile import SplineProfile` never fails
- `CommandInterface.run()` dispatches to the correct branch and raises a clean `NotImplementedError`
- Hardware tests for spline exist with `@pytest.mark.skip(reason="...Step 27...")`

### Windows serial port access
The Arduino IDE Serial Monitor holds the COM port exclusively. Close all Arduino IDE instances before running hardware tests.

### Windows console encoding
The `mock_demo.py` uses ASCII labels (`TX:` / `RX:`) instead of Unicode arrow characters (`→` / `←`) because Windows cmd/PowerShell defaults to cp1252 encoding which raises `UnicodeEncodeError`.

### pytest `hardware` mark registration
The `hardware` mark is registered in both `pyproject.toml` AND `tests/conftest.py` via `pytest_configure`. The conftest hook guarantees the mark is always registered regardless of pytest rootdir detection — eliminates `PytestUnknownMarkWarning` when running from any directory.

---

## Running the UI

The full UI (`main.py`) is not yet assembled. Two preview entry points exist now:

```bash
cd python

# Visual widget demo — EstopButton + StatusBar + LivePlot wired to MockArduino
uv run python widget_demo.py

# NOT YET BUILT — placeholder only:
uv run python main.py
```

`widget_demo.py` launches a standalone PyQt6 window with `MockArduino` running at 5× speed.
It automatically sequences HOME → RUN_PROFILE in a loop so you can see live telemetry.
Click EMERGENCY STOP at any time. No hardware required.

Once Steps 25–26 (main_window.py + main.py) are complete, the full tabbed UI will launch with:
```bash
cd python
uv run python main.py
```

---

## Running Tests

```bash
cd python

# All unit tests (no hardware):
uv run pytest -v

# Specific modules:
uv run pytest tests/ui/test_tabs.py -v        # control tab + serial monitor tab
uv run pytest tests/ui/test_widgets.py -v     # estop, live_plot, status_bar
uv run pytest tests/test_mock_arduino.py -v

# Hardware tests (Arduino on COM4):
uv run pytest -m hardware --hw-port COM4 -v

# Skip hardware tests explicitly:
uv run pytest -m "not hardware" -v
```

Current test counts (no hardware needed):
- `test_telemetry_parser.py` — ~30 tests
- `test_profile.py` — ~25 tests
- `test_command_interface.py` — ~30 tests
- `test_data_recorder.py` — ~20 tests
- `test_trapezoidal_profile.py` — ~25 tests
- `test_parabolic_profile.py` — ~20 tests
- `test_mock_arduino.py` — 51 tests
- `test_widgets.py` — 25 tests (pytest-qt, headless)
- `test_tabs.py` — 57 tests (pytest-qt, headless)
- **Total: 397 tests**

---

## Velocity Profile Architecture

Python does the maths; Arduino executes simple primitives (same principle as 3D printer slicers).

```python
class VelocityProfile(ABC):
    def get_speed_at(self, t: float) -> float: ...
    def total_duration(self) -> float: ...
    def to_segments(self, segment_length_mm: float = 1.0) -> list[MoveSegment]: ...
    def to_dict(self) -> ProfileDict: ...

    @classmethod
    def from_dict(cls, data: ProfileDict) -> VelocityProfile: ...
```

`CommandInterface.run()` dispatch table:

| `velocity_profile_type` | Wire path |
|---|---|
| `"trapezoidal"` | Single `CMD RUN_PROFILE` |
| `"segmented"` | `BEGIN_SEGMENTED_MOVE` → segments → `RUN_LOADED_MOVE` |
| `"parabolic"` | Same as segmented (profile converts to segments first) |
| `"spline"` | Raises `NotImplementedError` (Step 27) |

`to_segments()` must check `segment_count ≤ MOVE_SEG_BUFFER_SIZE (64)` and raise before touching serial.

---

## DipProfile Dataclass

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

`velocity_profile_data` contents by type:
- `"trapezoidal"` → `{}` empty
- `"segmented"` → `{"segments": [{"type": "move"|"dwell", ...}, ...]}`
- `"spline"` → `{"waypoints": [(pos_mm, speed_mm_s), ...]}` (future)

---

## UI Architecture

```
MainWindow (QMainWindow)
├── EstopButton                — always on top, calls ci.estop() directly
├── QTabWidget
│   ├── ControlTab             ✅ — connection, home, jog, run/pause/stop, profile picker
│   ├── SerialMonitorTab       ✅ — raw TX/RX log, multi-line input, autocomplete, history
│   ├── TelemetryTab           ⬜ — LivePlot + DataRecorder controls
│   ├── ProfileManagerTab      ⬜ — CRUD for JSON profiles in profiles/
│   └── ProfileEditorTab       ⬜ — stub (spline waypoint editor, Step 27)
└── StatusBar                  — port, Arduino state, phase, elapsed time
```

QTimer at 20 Hz polls `manager.telem_queue`, routes frames to StatusBar and TelemetryTab.
QTimer at 20 Hz in SerialMonitorTab polls `manager.raw_queue` for TX/RX log.
`CommandInterface.run()` must be called from a QThread (not the UI thread) — it blocks during segment streaming. `ControlTab._RunWorker` handles this.

---

## Coding Rules

1. **No `delay()` anywhere in Arduino firmware.** All timing via `millis()`.
2. **Serial thread never touches PyQt6 widgets.** Only signals/slots cross thread boundaries.
3. **Unit conversion only on the Arduino.** Wire and Python always speak mm.
4. **Every command gets ACK or ERR.** No silent failures.
5. **ESTOP bypasses all queues** — sent immediately, no ACK wait.
6. **`command_interface.run()` inspects `velocity_profile_type`** and dispatches automatically. UI never decides the wire path.
7. **`to_segments()` validates segment count ≤ 64** before any serial communication.
8. **Stubs (`spline_profile.py`, `profile_editor_tab.py`) have correct signatures** and raise `NotImplementedError` — never skip them.
9. **All unit tests run against `MockArduino`** — no test requires hardware except `tests/hardware/`.
10. **`logs/` and `profiles/` (user-created) are not committed.** `profiles/default.json` is the only committed profile.
11. **`uv.lock` is always committed.**
12. **One responsibility per module.**

---

## Package Management

Tool: `uv`

```bash
uv sync                  # install all dependencies
uv sync --extra dev      # include pytest, pytest-qt
uv run pytest            # run tests
uv run python main.py    # run app (once Step 26 is built)
```

Dependencies: `pyserial`, `PyQt6`, `pyqtgraph`, `numpy`, `scipy`
Dev dependencies: `pytest`, `pytest-qt`
