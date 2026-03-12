Dip Coater Control System — Full Architecture Brief
Project Overview
A dip coating rig controller with two components: Arduino firmware for the motion controller, and a Python desktop UI. The primary engineering priority is reproducible velocity profiles — speed and acceleration consistency matters more than absolute position accuracy.

Repository Structure
dip_coater/
├── .gitignore
├── README.md
├── ARCHITECTURE.md
├── arduino/
│   └── dip_coater_firmware/
│       ├── dip_coater_firmware.ino
│       ├── command_parser.h / .cpp
│       ├── state_machine.h / .cpp
│       ├── motion_controller.h / .cpp
│       ├── telemetry.h / .cpp
│       └── config.h
└── python/
    ├── pyproject.toml
    ├── uv.lock
    ├── .python-version
    ├── main.py
    ├── core/
    │   ├── serial_manager.py
    │   ├── command_interface.py
    │   ├── telemetry_parser.py
    │   ├── data_recorder.py
    │   └── profile.py
    ├── motion/
    │   ├── velocity_profile.py        # abstract base class
    │   ├── trapezoidal_profile.py     # current implementation
    │   └── spline_profile.py          # stub for future S-curve implementation
    ├── ui/
    │   ├── main_window.py
    │   ├── tabs/
    │   │   ├── control_tab.py
    │   │   ├── telemetry_tab.py
    │   │   ├── profile_manager_tab.py
    │   │   └── profile_editor_tab.py  # stub, shows placeholder
    │   └── widgets/
    │       ├── estop_button.py
    │       ├── live_plot.py
    │       └── status_bar.py
    ├── tests/
    │   ├── core/
    │   │   ├── test_command_interface.py
    │   │   ├── test_telemetry_parser.py
    │   │   └── test_profile.py
    │   ├── motion/
    │   │   └── test_velocity_profile.py
    │   └── mock_arduino.py
    ├── profiles/
    │   └── default.json
    └── logs/

Hardware

Microcontroller: uStepper S32
Motor: NEMA 23 stepper
Travel: ~900mm vertical axis
End switches: Two (top and bottom), active LOW with internal pull-up, on interrupt-capable pins
Library: uStepperS32.h — handles closed-loop PID and encoder feedback natively. The firmware sits on top of this library as a command interpreter and telemetry forwarder. Do not reimplement motion control — use the library.


Units Strategy

Arduino internally: steps (integer arithmetic, fast, matches uStepperS32 library natively)
Serial protocol (the wire): always mm, mm/s, mm/s² — human readable, no exceptions
Python internally: mm throughout, no conversion needed
Conversion lives only on the Arduino in config.h as STEPS_PER_MM
Arduino converts mm→steps on receipt of a command
Arduino converts steps→mm before broadcasting telemetry
Python never converts units. It uses telemetry values directly as mm.
unit_converter.py does not exist — it was removed because it is unnecessary and would be a source of drift bugs if it ever diverged from config.h


config.h — Already Written
cpp#define MOTOR_STEPS_PER_REV       200
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

#define DEBUG_DEFAULT_ON          0

// Segmented move buffer — max number of CMD MOVE_SEG segments Arduino will buffer
#define MOVE_SEG_BUFFER_SIZE      64
```

**These values need to be verified against physical hardware before first run:**
- `MICROSTEPS` — confirm S32 microstepping setting
- `LEADSCREW_MM_PER_REV` — confirm leadscrew pitch
- `PIN_ENDSTOP_BOTTOM` / `PIN_ENDSTOP_TOP` — confirm wiring

---

## State Machine — Already Written

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
Reported in telemetry so the Python UI can display the current phase to the operator.

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

### General rules
- All messages newline-terminated: `\n`
- Commands: Python → Arduino
- Telemetry and responses: Arduino → Python
- **All values on the wire in mm, mm/s, mm/s²** — no steps, no raw encoder counts
- Arduino converts to steps internally after parsing
- Arduino converts back to mm before sending telemetry

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

**Motion — simple profile (trapezoidal)**
```
CMD RUN_PROFILE <dip_speed_mm_s> <withdraw_speed_mm_s> <accel_mm_s2> <depth_mm> <dwell_bottom_ms> <dwell_top_ms> <n_dips>
```
Arduino executes the full dip cycle autonomously. Single command, Arduino owns the timing. Used for all trapezoidal profiles.

**Motion — segmented profile (spline / custom)**
```
CMD BEGIN_SEGMENTED_MOVE <n_segments> <n_dips> <dwell_bottom_ms> <dwell_top_ms>
CMD MOVE_SEG <distance_mm> <speed_mm_s>
CMD MOVE_SEG <distance_mm> <speed_mm_s>
... (repeated n_segments times, once per line)
CMD RUN_LOADED_MOVE
```
Python pre-computes the full velocity profile using scipy, samples it into short fixed-length segments, and streams the full segment list before triggering execution. Arduino buffers all segments then executes them as one continuous smooth move. The join between segments is handled by the uStepperS32 native acceleration so there are no jerky transitions. This is the same principle used by 3D printer firmware (Marlin/GRBL) — the host does the complex math, the firmware executes simple primitives.

### Responses (Arduino → Python)
```
ACK <COMMAND>                        # command accepted
ACK PROFILE_READY                    # all segments received, ready for RUN_LOADED_MOVE
ERR <COMMAND> <reason>               # command rejected with reason
ERR SEG_BUFFER_OVERFLOW              # too many segments for Arduino buffer
STATE <state> <phase>                # response to GET_STATE
```

### Telemetry broadcast (Arduino → Python, at configured rate)
```
TELEM,<millis>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_actual_mm_s2>,<state>,<phase>\n
```
Example:
```
TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING
Both commanded and actual velocity are always broadcast. This is critical — the S32 closed-loop PID means they may differ during ramps. Logging both lets you verify profile fidelity post-run.

Arduino Firmware Architecture
Four modules sitting on top of uStepperS32.h. The firmware is a command interpreter and telemetry forwarder — do not reimplement motion control.
command_parser.h/.cpp

Reads serial buffer character by character in loop()
On \n, tokenises and dispatches to the appropriate handler
Validates command against current state machine state
Returns ACK or ERR on every command — no silent failures
Never blocks — all parsing is non-blocking
Handles the multi-line segmented move handshake: after BEGIN_SEGMENTED_MOVE, parser enters a segment-collection mode and collects CMD MOVE_SEG lines until the declared count is reached, then sends ACK PROFILE_READY

state_machine.h/.cpp — Already written

Owns SystemState, RunPhase, ErrorCode
Exposes transition methods and canRun(), canPause(), canResume(), canJog() guards

motion_controller.h/.cpp

Owns the uStepperS32 instance
Executes HOME, RUN_PROFILE, BEGIN_SEGMENTED_MOVE/MOVE_SEG/RUN_LOADED_MOVE, JOG, STOP, ESTOP, PAUSE, RESUME
RUN_PROFILE: runs full dip cycle autonomously, non-blocking, millis()-based timing
RUN_LOADED_MOVE: executes buffered segments sequentially, each segment handed to uStepperS32 as a move-at-speed primitive, continuous execution with no gaps between segments
Calls state_machine.setPhase() as it transitions between DESCENDING / DWELL_BOTTOM / ASCENDING / DWELL_TOP
End switch ISRs call motion_controller.onEndstopTriggered() → state_machine.toError()
Pause: decelerates using same accel ramp as profile, saves remaining segments and current phase
Resume: re-accelerates, continues from saved segment index

telemetry.h/.cpp

Owns telem rate and last-broadcast timestamp
Called every loop() — checks interval, broadcasts if elapsed
Reads position and velocity from uStepperS32 encoder, converts to mm before sending
Reads commanded velocity from motion_controller
Formats and prints TELEM CSV line

dip_coater_firmware.ino

setup(): initialise serial, uStepperS32, pins, end switch interrupts, state machine
loop(): call command_parser update, motion_controller update, telemetry update — nothing else
No logic here beyond wiring the modules together

Critical rule: no delay() anywhere in the firmware. All timing via millis().

Python Architecture
Threading model — three threads

Serial thread: reads incoming lines, parses TELEM and ACK/ERR, puts onto queues. Never touches UI.
Data thread: consumes telemetry queue, appends to numpy arrays, triggers UI refresh, writes to log file.
Main/UI thread: PyQt5 event loop only. Reads from queues via signals/slots. Never blocks.

core/serial_manager.py

Owns the serial.Serial connection
Runs in serial thread
Emits parsed telemetry onto a queue.Queue
Emits ACK/ERR responses onto a separate queue.Queue
Exposes send_command(cmd_string) — thread-safe

core/command_interface.py

Builds and sends command strings from typed Python arguments
Inspects DipProfile.velocity_profile_type to choose the correct send path:

"trapezoidal" → sends single CMD RUN_PROFILE ... line
"spline" or any custom type → calls profile.to_segments(), streams CMD BEGIN_SEGMENTED_MOVE, all CMD MOVE_SEG lines, then CMD RUN_LOADED_MOVE


Awaits ACK/ERR from response queue with timeout on every command
All values passed and sent in mm — no unit conversion

python# Public interface examples
command_interface.home()
command_interface.run(profile: DipProfile)        # chooses RUN_PROFILE or segmented automatically
command_interface.stop()
command_interface.estop()                          # bypasses queue, sent immediately
command_interface.pause()
command_interface.resume()
command_interface.jog(direction: str, speed_mm_s: float)
command_interface.set_telem_rate(hz: int)
command_interface.set_soft_limits(min_mm: float, max_mm: float)
core/telemetry_parser.py

Parses raw TELEM,... strings into a TelemetryFrame dataclass

python@dataclass
class TelemetryFrame:
    timestamp_ms: int
    pos_mm: float
    vel_actual_mm_s: float
    vel_commanded_mm_s: float
    accel_actual_mm_s2: float
    state: str
    phase: str

Handles malformed lines gracefully — logs warning, does not crash

core/data_recorder.py

Maintains numpy arrays of all telemetry fields for the current run
Auto-saves CSV on run complete: logs/run_<profile_name>_<timestamp>.csv
Columns match all TelemetryFrame fields
Exposes current arrays to live plot widget

core/profile.py
python@dataclass
class DipProfile:
    name: str
    dip_speed_mm_s: float
    withdraw_speed_mm_s: float
    accel_mm_s2: float
    dip_depth_mm: float
    dwell_bottom_ms: int
    dwell_top_ms: int
    n_dips: int
    notes: str = ""
    created_at: str = ""
    velocity_profile_type: str = "trapezoidal"   # "trapezoidal" | "spline"
    velocity_profile_data: dict = field(default_factory=dict)  # spline waypoints go here

save_profile(profile, path) → JSON
load_profile(path) → DipProfile
list_profiles(directory) → list of DipProfile


Motion / Velocity Profile Architecture
This follows the same principle as 3D printer slicers: Python does the complex mathematics, Arduino executes simple move primitives. The abstract base class ensures trapezoidal and spline profiles are interchangeable everywhere in the codebase.
motion/velocity_profile.py — Abstract base class
python@dataclass
class MoveSegment:
    distance_mm: float
    speed_mm_s: float

class VelocityProfile(ABC):

    @abstractmethod
    def get_speed_at(self, t: float) -> float:
        """Return commanded speed mm/s at time t seconds into the move."""
        pass

    @abstractmethod
    def total_duration(self) -> float:
        """Return total move duration in seconds."""
        pass

    @abstractmethod
    def to_segments(self, segment_length_mm: float = 0.5) -> list[MoveSegment]:
        """
        Convert profile to short move segments for streaming to Arduino.
        Trapezoidal implements analytically.
        Spline samples the scipy interpolant.
        segment_length_mm controls resolution — shorter = smoother but more segments.
        Must not exceed MOVE_SEG_BUFFER_SIZE on the Arduino (64 segments default).
        """
        pass

    @abstractmethod
    def to_dict(self) -> dict:
        """Serialise for storage in DipProfile.velocity_profile_data."""
        pass

    @classmethod
    @abstractmethod
    def from_dict(cls, data: dict) -> 'VelocityProfile':
        pass
motion/trapezoidal_profile.py

Implements VelocityProfile
Standard trapezoidal: linear accel ramp → constant speed → linear decel ramp
Parameters: target_speed_mm_s, accel_mm_s2, distance_mm
to_segments() implemented analytically — no sampling needed
Also supports CMD RUN_PROFILE shortcut — command_interface uses this for trapezoidal profiles instead of streaming segments, since the Arduino can compute it natively

motion/spline_profile.py — Stub

Implements VelocityProfile
All methods raise NotImplementedError with message "Spline profile not yet implemented — use profile editor tab"
Class and method signatures must be complete and correct
Stub exists so imports never break and velocity_profile_type = "spline" is already handled in the dispatch logic


UI Architecture
Framework: PyQt5 with pyqtgraph
main_window.py

Top-level QMainWindow
Tab widget with four tabs
E-stop button always visible in toolbar above tabs — never hidden by tab switching
Status bar at bottom: connection state, Arduino state, current phase, elapsed run time

Tab 1 — Control (control_tab.py)

Profile parameter inputs: dip speed, withdraw speed, acceleration, dip depth, dwell bottom, dwell top, n_dips
Load / Save profile buttons
Home, Start, Pause/Resume, Stop buttons
Button states enforced by Arduino state machine:

Home: IDLE, READY, ERROR
Start: READY only
Pause: RUNNING only
Resume: PAUSED only
Stop: RUNNING, PAUSED


Soft limit pre-flight check before sending any run command — warn operator if depth exceeds limits

Tab 2 — Telemetry (telemetry_tab.py)

Numeric readouts: position, actual velocity, commanded velocity, current dip number, current phase
pyqtgraph live plot, three stacked subplots:

Position vs time (mm)
Velocity vs time (mm/s) — commanded in grey, actual in colour
Acceleration vs time (mm/s²)


Scrolling window during run, full session view post-run
Export plot as PNG button

Tab 3 — Profile Manager (profile_manager_tab.py)

List of saved JSON profiles in profiles/ directory
Columns: name, dip speed, withdraw speed, n_dips, created date, notes
Load selected profile into control tab
Duplicate, rename, delete
Compare two selected profiles side by side

Tab 4 — Profile Editor (profile_editor_tab.py) — STUB

Shows placeholder: "Advanced velocity profile editor — coming soon"
Will eventually allow drawing custom velocity vs time curves using linear segments or spline interpolation
Output saves into DipProfile.velocity_profile_data, sets velocity_profile_type = "spline"
Stub must exist with correct class signature so tab manager never breaks

widgets/estop_button.py

Large, red, always visible above tabs
On click: calls command_interface.estop() immediately, bypasses all queues

widgets/live_plot.py

Reusable pyqtgraph widget
Accepts a data_recorder reference
Updates on QTimer at 10Hz regardless of telemetry rate

widgets/status_bar.py

Serial connection status, Arduino state, current phase, elapsed run time


Testing Strategy
mock_arduino.py

Simulates the serial port — full Python-side development and testing without hardware
Responds to all CMD strings with correct ACK/ERR
Handles the full segmented move handshake: accepts BEGIN_SEGMENTED_MOVE, collects MOVE_SEG lines, sends ACK PROFILE_READY, executes on RUN_LOADED_MOVE
Broadcasts realistic TELEM strings at configurable rate
Can simulate error conditions: unexpected endstop, soft limit breach, buffer overflow
Used by all tests

Test priorities

test_telemetry_parser.py — valid lines, malformed lines, boundary values
test_command_interface.py — correct string construction, ACK/ERR handling, timeout, trapezoidal vs segmented dispatch
test_profile.py — save/load round-trip, missing fields, invalid values
test_velocity_profile.py — trapezoidal shape, duration, to_segments() output, segment count within buffer limit, edge cases

Running tests
bashcd python
uv run pytest
uv run pytest tests/core/test_telemetry_parser.py -v

Package Management

Tool: uv
Location: dip_coater/python/pyproject.toml

toml[project]
name = "dip-coater"
version = "0.1.0"
requires-python = ">=3.11"

dependencies = [
    "pyserial",
    "PyQt5",
    "pyqtgraph",
    "numpy",
    "scipy",
]

[project.optional-dependencies]
dev = [
    "pytest",
    "pytest-qt",
]
bashuv sync                  # install all dependencies
uv sync --extra dev      # include test dependencies
uv run main.py           # run app
uv run pytest            # run tests
```

---

## Coding Rules

1. **No `delay()` anywhere in the Arduino firmware.** All timing via `millis()`.
2. **Serial thread never touches PyQt5 widgets.** Only signals/slots cross thread boundaries.
3. **Unit conversion happens only on the Arduino.** Wire always speaks mm. Python never converts units.
4. **Every command gets an ACK or ERR.** No silent failures.
5. **E-stop bypasses all queues** — sent immediately and directly.
6. **`command_interface.run()` inspects `velocity_profile_type`** and dispatches to `CMD RUN_PROFILE` (trapezoidal) or the `BEGIN_SEGMENTED_MOVE` / `MOVE_SEG` / `RUN_LOADED_MOVE` sequence (spline/custom) automatically. The UI never decides which path to take.
7. **`to_segments()` must check segment count against `MOVE_SEG_BUFFER_SIZE`** (64) and raise a descriptive error if exceeded, before any serial communication begins.
8. **`profile_editor_tab.py` and `spline_profile.py` are stubs** — correct signatures, raise `NotImplementedError`, must not be skipped.
9. **All tests run against `mock_arduino.py`** — no test requires hardware.
10. **`logs/` and user `profiles/` are never committed** — in `.gitignore`. `profiles/default.json` is the only committed profile.
11. **`uv.lock` is always committed.**
12. **One responsibility per module.**

---

## Build Order
```
1.  config.h                          ✅ done
2.  state_machine.h/.cpp              ✅ done
3.  command_parser.h/.cpp
4.  motion_controller.h/.cpp
5.  telemetry.h/.cpp
6.  dip_coater_firmware.ino
7.  core/telemetry_parser.py          + tests
8.  core/profile.py                   + tests
9.  core/serial_manager.py
10. core/command_interface.py         + tests
11. core/data_recorder.py
12. motion/velocity_profile.py        (abstract base class + MoveSegment)
13. motion/trapezoidal_profile.py     + tests
14. motion/spline_profile.py          (stub)
15. mock_arduino.py
16. widgets/estop_button.py
17. widgets/live_plot.py
18. widgets/status_bar.py
19. ui/tabs/control_tab.py
20. ui/tabs/telemetry_tab.py
21. ui/tabs/profile_manager_tab.py
22. ui/tabs/profile_editor_tab.py     (stub)
23. ui/main_window.py
24. main.py

## Starting point

config.h and state_machine.h/.cpp are already written and committed to the repo.
Read those files first to understand the existing code before writing anything new.
Then work through the build order from step 3 onwards, one file at a time.
Write tests alongside each module as indicated. Do not skip stubs.

Now lets start step by step. Do you have any questions. Please clear everything before starting