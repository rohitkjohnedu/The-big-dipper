#include "state_machine.h"

// =============================================================================
// Constructor
// =============================================================================

StateMachine::StateMachine()
    : _state(SystemState::IDLE)
    , _phase(RunPhase::NONE)
    , _error(ErrorCode::NONE)
    , _homed(false)
{}

// =============================================================================
// Getters
// =============================================================================

SystemState StateMachine::getState() const { return _state; }
RunPhase    StateMachine::getPhase() const { return _phase; }
ErrorCode   StateMachine::getError() const { return _error; }
bool        StateMachine::isHomed()  const { return _homed; }

// =============================================================================
// State transitions
// =============================================================================

void StateMachine::toHoming() {
    _state = SystemState::HOMING;
    _phase = RunPhase::NONE;
    _error = ErrorCode::NONE;
}

void StateMachine::toReady() {
    _state = SystemState::READY;
    _phase = RunPhase::NONE;
    _homed = true;
}

void StateMachine::toRunning() { _state = SystemState::RUNNING; }
void StateMachine::toPaused()  { _state = SystemState::PAUSED;  }

void StateMachine::toIdle() {
    _state = SystemState::IDLE;
    _phase = RunPhase::NONE;
}

void StateMachine::toError(ErrorCode code) {
    _state = SystemState::ERROR;
    _phase = RunPhase::NONE;
    _error = code;
}

void StateMachine::setPhase(RunPhase phase) { _phase = phase; }

// =============================================================================
// Capability queries
// =============================================================================

bool StateMachine::canRun()    const { return _state == SystemState::READY;   }
bool StateMachine::canPause()  const { return _state == SystemState::RUNNING; }
bool StateMachine::canResume() const { return _state == SystemState::PAUSED;  }
bool StateMachine::canJog()    const { return _state == SystemState::READY;   }

// =============================================================================
// String helpers
// =============================================================================

const char* StateMachine::stateString() const {
    switch (_state) {
        case SystemState::IDLE:    return "IDLE";
        case SystemState::HOMING:  return "HOMING";
        case SystemState::READY:   return "READY";
        case SystemState::RUNNING: return "RUNNING";
        case SystemState::PAUSED:  return "PAUSED";
        case SystemState::ERROR:   return "ERROR";
        default:                   return "UNKNOWN";
    }
}

const char* StateMachine::phaseString() const {
    switch (_phase) {
        case RunPhase::NONE:         return "NONE";
        case RunPhase::DESCENDING:   return "DESCENDING";
        case RunPhase::DWELL_BOTTOM: return "DWELL_BOTTOM";
        case RunPhase::ASCENDING:    return "ASCENDING";
        case RunPhase::DWELL_TOP:    return "DWELL_TOP";
        default:                     return "UNKNOWN";
    }
}
