# Dip Coater — Session Continuity & Context Reference

Created: 2026-03-24
Purpose: Comprehensive onboarding and continuity reference for contributors
         and future Claude Code sessions. Captures decisions, rationale, and
         context that cannot be derived by reading the code alone.

---

## What This Project Is

A laboratory dip-coating rig controller. A motorised vertical stage dips a substrate
into a liquid bath at controlled speed and acceleration, holds it for a dwell period,
then withdraws. The key engineering requirement is **reproducible velocity profiles** —
the exact speed curve must be identical run-to-run for coating uniformity.

Two physical components:

1. **uStepper S32 (Arduino-based)** — runs firmware that interprets serial commands,
   executes motion using the built-in closed-loop PID, and streams telemetry back.

2. **Windows laptop** — runs a PyQt6 Python desktop application that connects over
   USB serial, sends commands, displays live plots, and saves CSV telemetry logs.

The system is a **research instrument** used in a laboratory setting, not a consumer
product. One operator, one rig, one COM port.

---

## Current Development State (2026-03-24)

### What is fully built and tested

**Arduino firmware** — all modules written and deployed to hardware:
- `config.h` — constants verified against physical hardware
- `state_machine.h/.cpp` — IDLE/HOMING/READY/RUNNING/PAUSED/ERROR
- `command_parser.h/.cpp` — non-blocking serial parser
- `motion_controller.h/.cpp` — all motion commands including segmented profiles
- `telemetry.h/.cpp` — periodic TELEM CSV broadcast
- `diagnostics.h/.cpp` — fault reporting

**Python core layer** — complete:
- `serial_manager.py` — serial port thread, three queues
- `command_interface.py` — typed Python API over serial protocol
- `telemetry_parser.py` — TELEM string → dataclass
- `data_recorder.py` — numpy arrays, CSV auto-save
- `profile.py` — DipProfile dataclass, JSON save/load

**Motion profiles** — trapezoidal and parabolic both hardware-tested:
- `trapezoidal_profile.py` — standard ramp, dispatched as single `CMD RUN_PROFILE`
- `parabolic_profile.py` — bell-curve v(x)=v_peak×4(x/d)(1-x/d), dispatched as segments
- `spline_profile.py` — stub only, all methods raise `NotImplementedError`

**MockArduino** — 51 tests, full state machine simulation

**UI widgets** (reusable, not tab-specific):
- `estop_button.py` — large red button, calls `ci.estop()` immediately
- `live_plot.py` — three pyqtgraph subplots (position, velocity, acceleration)
- `status_bar.py` — port indicator, state badge, phase, elapsed time

**UI tabs** (built so far):
- `control_tab.py` — connection, home/stop, jog, run/pause/resume, profile picker
- `serial_monitor_tab.py` — TX/RX log, multi-line command input, autocomplete, history

**Test suite: 397 non-hardware tests passing.**

### What remains to be built

```
Step 22  ui/tabs/telemetry_tab.py        — LivePlot + DataRecorder in a tab
Step 23  ui/tabs/profile_manager_tab.py  — CRUD for JSON profiles
Step 24  ui/tabs/profile_editor_tab.py   — stub (spline editor, future)
Step 25  ui/main_window.py               — assembles all tabs into QMainWindow
Step 26  main.py                         — entry point
Step 27  motion/spline_profile.py        — implement after UI complete
```

---

## How to Run the UI Right Now

The full UI (`main.py`) is not assembled yet. Two working previews exist:

```bash
cd python

# Widget demo — MockArduino at 5x speed, no hardware needed
# Shows: EstopButton + StatusBar + LivePlot cycling HOME → RUN_PROFILE → repeat
uv run python widget_demo.py

# Raw serial console demo — prints MockArduino TX/RX to terminal
uv run python mock_demo.py
```

`widget_demo.py` launches a real PyQt6 window. It requires a display (not headless).
On Windows, just double-click or run from a terminal. No hardware is needed — it uses
`MockArduino` internally.

Once `main_window.py` and `main.py` are built (Steps 25–26), the command will be:
```bash
cd python
uv run python main.py
```

The UI is a **local desktop application** — it runs on the Windows machine connected to
the rig. It is not a web application and has no browser interface.

---

## Architecture Decisions and Rationale

### Why MockArduino exists alongside hardware tests

This is the most frequently misunderstood aspect of the project. See also: CLAUDE.md.

**The short version:** MockArduino tests the Python layer in isolation. Hardware tests verify
the complete stack including firmware and UART. Both are necessary; neither replaces the other.

**The longer version:**

When you write `CommandInterface(mock)`, the `CommandInterface` code is completely unaware
it is talking to a simulation. `send_command("CMD HOME")` writes to `mock.raw_queue` and
gets `"ACK HOME"` back on `mock.response_queue`, exactly as it would with a real serial port.

This means:
- You can test all Python logic (protocol correctness, error handling, state transitions,
  UI button states) without the rig being present
- Tests run in <30 seconds total (MockArduino supports speed multiplier, default 5×)
- Tests are deterministic — MockArduino always starts in IDLE state with no prior history
- You can inject faults that are impossible on real hardware (simulated endstop trigger
  during a move, buffer overflow, ERR responses on specific commands)
- CI/CD becomes possible — tests pass on any machine with Python installed

Hardware tests (`pytest -m hardware --hw-port COM4`) exist to verify things MockArduino
cannot simulate: the real UART timing, the actual stepper acceleration curve, the physical
endstop response, and firmware bugs.

**Rule:** Every new feature should first be tested with MockArduino. Hardware tests are
validation, not development.

### Why the serial protocol uses mm throughout (no steps on the wire)

The uStepperS32 library works in steps internally. The Python code works in mm. The serial
wire always uses mm.

The alternative — passing steps on the wire — would require the Python code to know
`STEPS_PER_MM`. That constant lives in `config.h` on the Arduino. If someone changes
the leadscrew, updates `config.h`, but forgets to update a Python constant, every move
command would be wrong by an invisible scaling factor. That class of bug is very hard to
debug because the motor moves, just by the wrong amount.

By putting the conversion exclusively on the Arduino, there is one source of truth.
`unit_converter.py` does not exist and must never be created.

### Why CommandInterface.run() decides the wire path, not the UI

The UI passes a `DipProfile` object to `CommandInterface.run()`. The command interface
inspects `profile.velocity_profile_type` and decides whether to send `CMD RUN_PROFILE`
(trapezoidal) or the `BEGIN_SEGMENTED_MOVE` streaming sequence (parabolic/segmented).

The UI does not know which path is taken. This is intentional: if a new profile type is
added in the future, only `command_interface.py` needs to change — no UI code is touched.

### Why `_RunWorker(QThread)` runs CommandInterface.run()

`CommandInterface.run()` for segmented profiles blocks while streaming dozens of
`CMD MOVE_SEG` commands, each followed by a 10 ms inter-segment delay. For a parabolic
profile with 40 segments that is 400 ms of blocking. Running this on the UI thread would
freeze the application and prevent ESTOP from responding.

`ControlTab._RunWorker` is a `QThread` subclass that runs `ci.run(profile)` in a
background thread and emits `finished` or `error(str)` signals when complete. The UI
thread stays responsive throughout.

ESTOP is special: `ci.estop()` is NOT routed through the run worker. It calls
`serial_manager.send_command("CMD ESTOP")` directly from whatever thread calls it,
bypassing all queues and all waits.

### Why raw_queue was added to both SerialManager and MockArduino

The serial monitor tab needs to display every TX and RX line. The existing `telem_queue`
and `response_queue` are consumed by `CommandInterface` and the telemetry polling loop —
tapping into those would require coordination and could affect timing.

Instead, `raw_queue` receives a copy of every line as a plain string:
- `"TX CMD HOME"` when a command is sent
- `"RX ACK HOME"` when a response arrives
- `"RX TELEM,12453,45.32,..."` when a telemetry frame arrives

The monitor tab drains `raw_queue` at 20 Hz in its own `QTimer`. Existing consumers are
completely unaffected. The queue is bounded at 2000 entries — if the monitor tab is not
being displayed, entries are silently dropped rather than growing unbounded.

### Why the inter-segment delay is 10 ms

The Arduino Mega/S32 has a hardware UART with a 64-byte RX buffer. At 115200 baud,
that buffer fills in roughly 5 ms if data arrives continuously. A `CMD MOVE_SEG` line
is ~25 bytes. Sending them back-to-back with no delay would overflow the buffer
around segment 3–4, causing partial lines, parse errors, and `ERR SEG_BUFFER_OVERFLOW`.

10 ms gives the Arduino time to parse each segment and write it to its segment buffer
before the next one arrives. This was determined empirically during hardware testing.
The delay is defined as `INTER_SEGMENT_DELAY_S = 0.010` in `command_interface.py`.

### Why SplineProfile is a stub with NotImplementedError

The spline editor (Step 27) requires a custom velocity curve drawing UI that does not
exist yet. However, the class and all method signatures must exist right now so that:

1. `from motion.spline_profile import SplineProfile` never raises `ImportError`
2. `CommandInterface.run()` can include the dispatch branch for `"spline"` type
   without an `AttributeError` if someone loads a future profile
3. Hardware tests for spline can be written now (with `@pytest.mark.skip`) so the
   test structure is ready when the implementation arrives

The stub raises `NotImplementedError` with a clear message — this is a deliberate and
documented choice, not missing code.

### Why pytest hardware mark is registered in both pyproject.toml and conftest.py

pytest resolves `pyproject.toml` relative to the rootdir, which it detects by scanning
upward from the test file location. If tests are run from a subdirectory (e.g.,
`tests/hardware/`) rather than from `python/`, pytest may not find `pyproject.toml`
and the `hardware` mark registration would be missing, causing `PytestUnknownMarkWarning`.

The `pytest_configure` hook in `tests/conftest.py` always runs regardless of rootdir
detection, so the mark is registered unconditionally.

### Why the serial monitor supports multi-line input

The original Arduino IDE serial monitor sends one command at a time. For common sequences
(HOME, then SET_TELEM_RATE 10, then JOG UP 5.0) this requires three separate sends.

The serial monitor tab uses `QPlainTextEdit` (not `QLineEdit`) for input, allowing the
operator to type multiple commands separated by newlines. Ctrl+Enter sends all lines
sequentially. This matches how bash and other terminal tools work. The History sidebar
stores each sent batch as a unit, so a multi-command sequence can be recalled and
re-sent with one click.

---

## Key Invariants (things that must never change)

1. **Wire protocol always uses mm** — no steps, no raw encoder values
2. **ESTOP bypasses all queues and delays** — must reach the Arduino immediately
3. **No `delay()` in Arduino firmware** — everything uses `millis()`
4. **Serial thread never touches Qt widgets** — crossing threads crashes PyQt6
5. **`to_segments()` validates count ≤ 64 before any serial write**
6. **MockArduino interface must match SerialManager exactly** — any divergence breaks tests
7. **Stubs must have correct signatures** — they must be importable and callably-typed

---

## File Ownership Map

| File | Purpose | Who calls it |
|------|---------|--------------|
| `serial_manager.py` | Owns the serial port | `MainWindow` (creates, calls start/stop) |
| `command_interface.py` | Typed command API | `ControlTab`, `EstopButton`, `_RunWorker` |
| `telemetry_parser.py` | Parse TELEM strings | `SerialManager._read_loop()` |
| `data_recorder.py` | Log telemetry to CSV | `TelemetryTab` (Step 22) |
| `profile.py` | DipProfile JSON I/O | `ProfileManagerTab`, `ControlTab` |
| `mock_arduino.py` | Test-only simulator | All non-hardware tests |
| `estop_button.py` | ESTOP widget | `MainWindow` toolbar |
| `live_plot.py` | pyqtgraph plots | `TelemetryTab`, `widget_demo.py` |
| `status_bar.py` | State/phase/elapsed | `MainWindow` bottom bar |
| `control_tab.py` | Operator controls | `MainWindow` tab 1 |
| `serial_monitor_tab.py` | TX/RX log | `MainWindow` tab 2 |

---

## What "Reproducible Velocity Profiles" Means in Practice

The primary engineering requirement drives several design choices that may seem
over-engineered for a simple motor controller:

- **Parabolic profile** — a linear ramp would cause jerk (instantaneous acceleration
  change) at the start/end of the move, which could affect coating uniformity. The
  bell-curve profile `v(x) = v_peak × 4(x/d)(1−x/d)` has zero velocity at both ends
  and a smooth peak, eliminating jerk. Hardware-tested and confirmed.

- **Segmented streaming** — rather than telling the Arduino "move at 8 mm/s", we send
  30–40 short segments each with a slightly different speed, pre-computed in Python.
  This is identical to how 3D printer slicers work. Python has the full scientific stack
  (numpy, scipy) to compute the curve; the Arduino just executes simple primitives.

- **Commanded vs actual velocity both broadcast in TELEM** — the uStepperS32 PID means
  the actual velocity may lag the commanded velocity during ramps. Broadcasting both
  lets us verify post-run that the motor actually followed the profile.

- **DataRecorder auto-saves CSV** — every run is logged automatically. Post-run analysis
  in Python/MATLAB can compare profiles across experiments.

---

## Development Workflow

```bash
cd python

# 1. Run all tests before any change
uv run pytest -q

# 2. Make changes

# 3. Run tests again — all 397 must pass
uv run pytest -q

# 4. Visual check (no hardware needed)
uv run python widget_demo.py

# 5. Hardware validation (only when rig is on bench)
uv run pytest -m hardware --hw-port COM4 -v
```

**Do not commit if any test fails.** The full test suite runs in under 30 seconds.

---

## Glossary

| Term | Meaning |
|------|---------|
| Dip cycle | One complete descend → dwell → ascend → dwell sequence |
| n_dips | Number of consecutive dip cycles in one profile run |
| Soft limit | Software-enforced position boundary (10–890 mm), below the endstops |
| Segment | A short `(distance_mm, speed_mm_s)` move primitive streamed to Arduino |
| TELEM frame | One line of periodic telemetry from Arduino, parsed into `TelemetryFrame` |
| ACK | Arduino confirmation that a command was accepted |
| ERR | Arduino rejection of a command with a reason string |
| Phase | Sub-state during RUNNING: DESCENDING, DWELL_BOTTOM, ASCENDING, DWELL_TOP |
| raw_queue | Queue of plain-string copies of all TX/RX lines, fed to serial monitor |
| MockArduino | Python Arduino simulator — same public API as `SerialManager` |
| `_RunWorker` | `QThread` subclass in `ControlTab` that runs `ci.run()` off the UI thread |
