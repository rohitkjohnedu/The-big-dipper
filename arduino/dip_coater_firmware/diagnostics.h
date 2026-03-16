#pragma once
#include <Arduino.h>
#include "motion_controller.h"
#include "state_machine.h"

// =============================================================================
// Diagnostics
//
// Activated by serial command from the main loop.
// All motor tests run non-blocking through update().
//
// Commands:
//   DIAG MOTOR    — motor connectivity + accuracy test sequence
//   DIAG ENDSTOP  — live endstop monitoring (trigger manually)
//   DIAG EXIT     — return to standby
// =============================================================================

class Diagnostics {
public:
    enum class Mode : uint8_t { INACTIVE, MOTOR_TEST, ENDSTOP_TEST };

    Diagnostics();
    void begin(MotionController& mc, StateMachine& sm);

    void startMotorTest();
    void startEndstopTest();
    void exit();
    void update();

    Mode mode() const { return _mode; }
    bool active() const { return _mode != Mode::INACTIVE; }

private:
    enum class MotorPhase : uint8_t {
        JOG_DOWN,
        JOG_DOWN_SETTLE,
        JOG_DOWN_CHECK,
        HOME_CMD,
        HOME_WAIT,
        HOME_CHECK,
        ACCURACY_DOWN,
        ACCURACY_DOWN_SETTLE,
        ACCURACY_DOWN_CHECK,
        ACCURACY_UP,
        ACCURACY_UP_SETTLE,
        ACCURACY_UP_CHECK,
        DONE
    };

    MotionController* _mc;
    StateMachine*     _sm;
    Mode              _mode;
    MotorPhase        _motorPhase;
    uint32_t          _phaseStart;
    float             _startPos;
    int               _passed;
    int               _failed;

    // Endstop debounce tracking
    bool _lastTop;
    bool _lastBot;

    void check(const char* name, bool ok);
    void updateMotorTest();
    void updateEndstopTest();

    // Test parameters
    static constexpr float    JOG_SPEED        =  5.0f;
    static constexpr uint32_t PRESENCE_MS      =  2000;
    static constexpr float    PRESENCE_MIN_MM  =  3.0f;
    static constexpr uint32_t HOME_TIMEOUT_MS  = 30000;
    static constexpr uint32_t ACCURACY_DOWN_MS = 10000;
    static constexpr uint32_t ACCURACY_UP_MS   =  5000;
    static constexpr float    TOLERANCE_MM     =  2.0f;
    static constexpr uint32_t SETTLE_MS        =   500;
    static constexpr uint32_t DEBOUNCE_MS      =    20;
};
