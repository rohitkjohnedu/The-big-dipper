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
    enum class Mode : uint8_t { INACTIVE, MOTOR_TEST, ENDSTOP_TEST, JOG, MOVE_TEST, CAL_MOVE };

    Diagnostics();
    void begin(MotionController& mc, StateMachine& sm);

    void startMotorTest();
    void startMotorEncoderTest();
    void startEndstopTest();
    void startJog(bool up, float speedMms);
    void startMoveTest(float mm);
    void startCalMove(float mm);
    void computeCalResult(float actualMm);
    void printPosition();
    void exit();
    void update();

    Mode mode() const { return _mode; }
    bool active() const { return _mode != Mode::INACTIVE; }

private:
    enum class MotorPhase : uint8_t {
        // Simple move test (current)
        MOVE_DOWN,
        MOVE_DOWN_WAIT,
        MOVE_UP,
        MOVE_UP_WAIT,
        DONE,
        // Extended test phases (reserved for later)
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
        ACCURACY_UP_CHECK
    };

    MotionController* _mc;
    StateMachine*     _sm;
    Mode              _mode;
    MotorPhase        _motorPhase;
    uint32_t          _phaseStart;
    float             _startPos;
    int               _passed;
    int               _failed;
    bool              _checkEncoder;

    // Endstop debounce tracking
    bool _lastTop;
    bool _lastBot;

    // Move test state
    float _moveTestStartPos;
    float _moveTestDeltaMm;
    uint32_t _moveTestStart;

    void check(const char* name, bool ok);
    void updateMotorTest();
    void updateEndstopTest();
    void updateMoveTest();
    void updateCalMove();

    // Calibration state — holds results across DIAG CAL RESULT
    float    _calStartPos;
    float    _calCommandedMm;
    float    _calEncoderMm;    // encoder displacement, positive = down (for display)
    uint32_t _calStart;

    // Test parameters — speeds kept low to limit current draw
    static constexpr float    DIAG_SPEED_MMS   =  2.0f;
    static constexpr float    CAL_SPEED_MMS    = 10.0f;   // faster for calibration moves
    static constexpr float    DIAG_ACCEL_MMS2  =  5.0f;
    static constexpr float    DIAG_DIST_MM     =  5.0f;
    static constexpr float    TOLERANCE_MM     =  1.5f;
    static constexpr uint32_t MOVE_TIMEOUT_MS  = 30000;
    static constexpr uint32_t SETTLE_MS        =   500;
    static constexpr uint32_t DEBOUNCE_MS      =    20;
    // Reserved for extended tests
    static constexpr float    JOG_SPEED        =  2.0f;
    static constexpr uint32_t PRESENCE_MS      =  2000;
    static constexpr float    PRESENCE_MIN_MM  =  3.0f;
    static constexpr uint32_t HOME_TIMEOUT_MS  = 30000;
    static constexpr uint32_t ACCURACY_DOWN_MS = 10000;
    static constexpr uint32_t ACCURACY_UP_MS   =  5000;
};
