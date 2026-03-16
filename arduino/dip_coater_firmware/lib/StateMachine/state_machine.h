#pragma once

#ifdef ARDUINO
#  include <Arduino.h>   // uint8_t for embedded builds
#else
#  include <cstdint>     // uint8_t for native/host builds
#endif

// =============================================================================
// state_machine.h — System states and transition logic
// =============================================================================


// -----------------------------------------------------------------------------
// State enum
// -----------------------------------------------------------------------------
enum class SystemState : uint8_t {
    IDLE,       // Powered on, not homed. No motion commands accepted except HOME.
    HOMING,     // Driving toward bottom end switch to zero position.
    READY,      // Homed and stationary. Accepts all commands.
    RUNNING,    // Executing a dip profile.
    PAUSED,     // Mid-profile pause. Position held. Can RESUME or STOP.
    ERROR       // Fault condition. Requires reset or HOME to recover.
};


// -----------------------------------------------------------------------------
// Phase enum — sub-state during RUNNING / PAUSED
// Reported in telemetry so Python can show current phase to the operator.
// -----------------------------------------------------------------------------
enum class RunPhase : uint8_t {
    NONE,           // Not in a run
    DESCENDING,     // Moving down toward the solution
    DWELL_BOTTOM,   // Stationary in solution
    ASCENDING,      // Withdrawing from solution
    DWELL_TOP,      // Stationary at top, waiting between dips
};


// -----------------------------------------------------------------------------
// Error codes
// -----------------------------------------------------------------------------
enum class ErrorCode : uint8_t {
    NONE,
    ENDSTOP_TRIGGERED_UNEXPECTEDLY,
    SOFT_LIMIT_EXCEEDED,
    COMMAND_INVALID_STATE,      // Command received but not valid in current state
    COMMAND_PARSE_ERROR,        // Malformed command string
    PROFILE_INVALID,            // Profile parameters out of range
    SEG_BUFFER_OVERFLOW,        // More MOVE_SEG segments than MOVE_SEG_BUFFER_SIZE
};


// -----------------------------------------------------------------------------
// StateMachine class
// -----------------------------------------------------------------------------
class StateMachine {
public:
    StateMachine();

    // Current state accessors
    SystemState getState() const;
    RunPhase    getPhase() const;
    ErrorCode   getError() const;

    // State transition methods — called by command_parser and motion_controller
    void toHoming();
    void toReady();
    void toRunning();
    void toPaused();
    void toIdle();
    void toError(ErrorCode code);

    // Phase transitions — called by motion_controller during profile execution
    void setPhase(RunPhase phase);

    // Convenience queries
    bool isHomed()   const;
    bool canRun()    const;   // true if READY
    bool canPause()  const;   // true if RUNNING
    bool canResume() const;   // true if PAUSED
    bool canJog()    const;   // true if READY

    // Human-readable strings for telemetry output
    const char* stateString() const;
    const char* phaseString() const;

private:
    SystemState _state;
    RunPhase    _phase;
    ErrorCode   _error;
    bool        _homed;
};