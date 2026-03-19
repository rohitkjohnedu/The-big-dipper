#pragma once
#include <Arduino.h>
#include "motion_controller.h"
#include "state_machine.h"

/**
 * @file  diagnostics.h
 * @brief Hardware self-test and calibration routines for the dip coater.
 *
 * Diagnostics is activated by serial commands and runs all motor tests
 * non-blocking through update().  Each test is started by a dedicated
 * startXxx() method and advances through an internal state machine until
 * it completes or times out.
 *
 * Command surface
 * ---------------
 *   DIAG MOTOR              — move down then up, print completion
 *   DIAG MOTORENCODER       — same as MOTOR, plus encoder pass/fail check
 *   DIAG ENDSTOP            — live endstop monitor (trigger manually)
 *   DIAG MOVE <mm> [spd] [accel] — single move with PASS/FAIL report
 *   DIAG JOG <UP|DOWN> [spd]     — continuous velocity jog
 *   DIAG CAL <mm>           — calibration move: command distance, read encoder
 *   DIAG CAL RESULT <mm>    — enter measured distance, compute correction factor
 *   DIAG POS                — print current position (mm)
 *   DIAG EXIT               — stop any active test and return to standby
 */

class Diagnostics {
public:

    /** @brief Active diagnostic mode. */
    enum class Mode : uint8_t { INACTIVE, MOTOR_TEST, ENDSTOP_TEST, JOG, MOVE_TEST, CAL_MOVE };

    Diagnostics();

    /**
     * @brief Wire up the shared MotionController and StateMachine.
     *        Must be called once from setup() before any other method.
     */
    void begin(MotionController& mc, StateMachine& sm);

    // -------------------------------------------------------------------------
    // Test launchers — called by CommandParser
    // -------------------------------------------------------------------------

    /** @brief Move down DIAG_DIST_MM then back up; report completion. */
    void startMotorTest();

    /** @brief Same as startMotorTest() but also performs an encoder accuracy check. */
    void startMotorEncoderTest();

    /** @brief Enter live endstop monitor mode; prints each state change. */
    void startEndstopTest();

    /**
     * @brief Start a continuous velocity jog.
     * @param up       true = upward, false = downward.
     * @param speedMms Jog speed (mm/s).
     */
    void startJog(bool up, float speedMms);

    /**
     * @brief Command a single move and report PASS/FAIL against TOLERANCE_MM.
     * @param mm        Signed displacement (mm, +ve = up).
     * @param speedMms  Travel speed (mm/s).
     * @param accelMms2 Acceleration (mm/s²).
     */
    void startMoveTest(float mm,
                       float speedMms  = DIAG_SPEED_MMS,
                       float accelMms2 = DIAG_ACCEL_MMS2);

    /**
     * @brief Command a move at CAL_SPEED_MMS and record the encoder displacement.
     *        Follow with computeCalResult() after measuring the physical displacement.
     * @param mm Signed displacement to command (mm).
     */
    void startCalMove(float mm);

    /**
     * @brief Compute and print a leadscrew correction factor.
     * @param actualMm Physical displacement measured with calipers (mm).
     *                 Compares against the encoder reading from the last startCalMove().
     */
    void computeCalResult(float actualMm);

    /** @brief Print the current encoder position (mm) to Serial. */
    void printPosition();

    /** @brief Stop any active test/jog and return to INACTIVE. */
    void exit();

    /** @brief Service the active diagnostic mode.  Call every loop(). */
    void update();

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /** @brief Returns the current diagnostic mode. */
    Mode mode()   const { return _mode; }

    /** @brief Returns true while any diagnostic test is active. */
    bool active() const { return _mode != Mode::INACTIVE; }

    // -------------------------------------------------------------------------
    // Public constants (referenced by CommandParser for default arguments)
    // -------------------------------------------------------------------------

    static constexpr float DIAG_SPEED_MMS  =  2.0f;   ///< Default move test speed (mm/s)
    static constexpr float DIAG_ACCEL_MMS2 =  5.0f;   ///< Default move test acceleration (mm/s²)
    static constexpr float CAL_SPEED_MMS   = 10.0f;   ///< Calibration move speed (mm/s)

private:

    // -------------------------------------------------------------------------
    // Internal types
    // -------------------------------------------------------------------------

    /**
     * @brief Sub-phases for updateMotorTest().
     *
     * MOVE_DOWN / MOVE_UP are the active phases.  The remaining phases
     * (JOG_DOWN … ACCURACY_UP_CHECK) are reserved for a future extended
     * motor characterisation test and are not yet exercised.
     */
    enum class MotorPhase : uint8_t {
        // Simple move test (current)
        MOVE_DOWN,
        MOVE_UP,
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
        ACCURACY_UP_CHECK,
    };

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

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
    float    _moveTestStartPos;
    float    _moveTestDeltaMm;
    float    _moveTestSpeedMms;
    float    _moveTestAccelMms2;
    uint32_t _moveTestStart;

    // Movement detection — used by updateMotorTest() and updateCalMove()
    bool     _moveStarted;   ///< true once position has left start by > MOVE_START_MM
    float    _trackedPos;    ///< last position at which significant movement was detected
    uint32_t _stableSince;   ///< millis() when position was last updated significantly

    // Calibration state — holds results across DIAG CAL RESULT
    float    _calStartPos;
    float    _calCommandedMm;
    float    _calEncoderMm;  ///< encoder displacement (positive = down, for display)
    uint32_t _calStart;

    // -------------------------------------------------------------------------
    // Private constants
    // -------------------------------------------------------------------------

    static constexpr float    DIAG_DIST_MM     =   5.0f;   ///< Distance for DIAG MOTOR test (mm)
    static constexpr float    TOLERANCE_MM     =   1.5f;   ///< Acceptable position error for PASS/FAIL (mm)
    static constexpr uint32_t MOVE_TIMEOUT_MS  = 60000;    ///< Minimum move timeout — updateMoveTest scales up with distance/speed (ms)
    static constexpr uint32_t SETTLE_MS        =   300;    ///< Position stability window for motor-test completion (ms)
    static constexpr float    MOVE_START_MM    =   0.2f;   ///< Movement detection threshold (mm)
    static constexpr float    STABLE_MM        =   0.05f;  ///< Max drift that counts as settled (mm)
    static constexpr uint32_t DEBOUNCE_MS      =    20;    ///< Endstop debounce window (ms)

    // Reserved constants for extended motor test
    static constexpr float    JOG_SPEED        =   2.0f;   ///< Speed for extended jog phase (mm/s)
    static constexpr uint32_t PRESENCE_MS      =  2000;    ///< Jog duration for presence check (ms)
    static constexpr float    PRESENCE_MIN_MM  =   3.0f;   ///< Minimum encoder travel to confirm motor presence (mm)
    static constexpr uint32_t HOME_TIMEOUT_MS  = 30000;    ///< Timeout waiting for homing to complete (ms)
    static constexpr uint32_t ACCURACY_DOWN_MS = 10000;    ///< Accuracy jog down duration (ms)
    static constexpr uint32_t ACCURACY_UP_MS   =  5000;    ///< Accuracy jog up duration (ms)

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** @brief Print a DIAG:PASS or DIAG:FAIL line and increment pass/fail counters. */
    void check(const char* name, bool ok);

    // Mode-specific update handlers (called from update())
    void updateMotorTest();
    void updateEndstopTest();
    void updateMoveTest();
    void updateCalMove();
};
