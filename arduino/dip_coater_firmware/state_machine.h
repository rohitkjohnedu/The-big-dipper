#pragma once
#include <Arduino.h>

// =============================================================================
// state_machine.h — System states and transition logic
// =============================================================================

enum class SystemState : uint8_t {
    IDLE,       // Powered on, not homed. Only HOME command accepted.
    HOMING,     // Driving toward top endstop to zero position.
    READY,      // Homed and stationary. Accepts all commands.
    RUNNING,    // Executing a dip profile.
    PAUSED,     // Mid-profile pause. Can RESUME or STOP.
    ERROR       // Fault condition. Requires HOME or reset to recover.
};

enum class RunPhase : uint8_t {
    NONE,
    DESCENDING,     // Moving down toward solution
    DWELL_BOTTOM,   // Stationary in solution
    ASCENDING,      // Withdrawing from solution
    DWELL_TOP,      // Stationary at top, waiting between dips
};

enum class ErrorCode : uint8_t {
    NONE,
    ENDSTOP_TRIGGERED_UNEXPECTEDLY,
    SOFT_LIMIT_EXCEEDED,
    COMMAND_INVALID_STATE,
    COMMAND_PARSE_ERROR,
    PROFILE_INVALID,
    SEG_BUFFER_OVERFLOW,
};

class StateMachine {
public:
    StateMachine();

    SystemState getState() const;
    RunPhase    getPhase() const;
    ErrorCode   getError() const;

    void toHoming();
    void toReady();
    void toRunning();
    void toPaused();
    void toIdle();
    void toError(ErrorCode code);

    void setPhase(RunPhase phase);

    bool isHomed()   const;
    bool canRun()    const;
    bool canPause()  const;
    bool canResume() const;
    bool canJog()    const;

    const char* stateString() const;
    const char* phaseString() const;

private:
    SystemState _state;
    RunPhase    _phase;
    ErrorCode   _error;
    bool        _homed;
};
