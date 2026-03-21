#pragma once
#include <Arduino.h>
#include <UstepperS32.h>
#include "config.h"
#include "state_machine.h"

/**
 * @file  motion_controller.h
 * @brief All stepper motion for the dip coater.
 *
 * MotionController owns the UstepperS32 instance and is the single place
 * that issues commands to the motor.  Every public method is non-blocking —
 * call begin() once from setup() and update() on every loop() iteration.
 *
 * Coordinate system
 * -----------------
 *   Home (top endstop) = 0 mm
 *   Positive mm        = upward   (toward home)
 *   Negative mm        = downward (into solution)
 *   CW rotation        = up,  CCW = down  (swap in begin() if wiring is reversed)
 *
 * Motion modes
 * ------------
 *   HOMING        — drives upward until the top endstop fires, then backs off
 *   TRAPEZOIDAL   — executes a multi-dip profile (descent / dwell / ascent / dwell)
 *   SEGMENTED     — executes a pre-loaded sequence of variable-speed segments
 *   JOG           — continuous velocity, no target
 *   MOVING        — single relative move (used by Diagnostics and CMD MOVE)
 *   LIMIT_BACKOFF — backs away from an endstop after an unexpected trigger
 */

class MotionController {
public:

    /** @brief Construct with a reference to the shared StateMachine. */
    explicit MotionController(StateMachine& sm);

    /** @brief Initialise the stepper driver.  Call once from setup(). */
    void begin();

    /** @brief Service the active motion mode.  Call every loop(). */
    void update();

    // -------------------------------------------------------------------------
    // Commands — called by CommandParser
    // -------------------------------------------------------------------------

    /** @brief Start the homing sequence: drive up until top endstop, then back off. */
    void executeHome();

    /**
     * @brief Override the soft-limit window at runtime.
     * @param minMm  Most-negative allowed position (mm, must be < maxMm).
     * @param maxMm  Most-positive allowed position (mm).
     */
    void setSoftLimits(float minMm, float maxMm);

    /** @brief Soft-stop the motor and transition to READY. */
    void stop();

    /** @brief Hard-stop the motor and transition to ERROR. */
    void estop();

    /** @brief Snapshot the profile state and soft-stop.  Resume with resume(). */
    void pause();

    /** @brief Restart the profile from the snapshotted state after a pause(). */
    void resume();

    /**
     * @brief Start continuous velocity jog.
     * @param up       true = upward, false = downward.
     * @param speedMms Jog speed (mm/s).
     */
    void jog(bool up, float speedMms);

    /**
     * @brief Start a trapezoidal dip profile.
     * @param dipSpeedMms      Descent speed (mm/s).
     * @param withdrawSpeedMms Withdrawal speed (mm/s).
     * @param accelMms2        Acceleration and deceleration (mm/s²).
     * @param depthMm          Dip depth below the current position (mm, positive value).
     * @param dwellBottomMs    Time to dwell in the solution (ms).
     * @param dwellTopMs       Time to dwell at the top between dips (ms).
     * @param nDips            Number of dip cycles to execute.
     */
    void runProfile(float    dipSpeedMms, float withdrawSpeedMms,
                    float    accelMms2,   float depthMm,
                    uint32_t dwellBottomMs, uint32_t dwellTopMs, int nDips);

    /**
     * @brief Reset the segment buffer ready for addSegment() calls.
     *        Must be called before loading a new segment sequence.
     */
    void beginSegmentedMove();

    /**
     * @brief Append a motion segment to the loaded move buffer.
     * @param distMm    Signed displacement (mm, +ve = up).
     * @param speedMms  Speed (mm/s).
     * @param accelMms2 Acceleration (mm/s²).
     * @return true on success, false if the buffer is full.
     */
    bool addSegment(float distMm, float speedMms, float accelMms2);

    /**
     * @brief Append a dwell (hold-position) segment to the loaded move buffer.
     * @param dwellMs  Time to hold position (ms).
     * @return true on success, false if the buffer is full.
     */
    bool addDwellSegment(uint32_t dwellMs);

    /** @brief Execute the previously loaded segment sequence. */
    void runLoadedMove();

    /**
     * @brief Command a single relative move.  Used by Diagnostics and CMD MOVE.
     * @param deltaMm   Signed displacement (mm, +ve = up).
     * @param speedMms  Travel speed (mm/s).
     * @param accelMms2 Acceleration (mm/s²).
     */
    void moveByMm(float deltaMm, float speedMms, float accelMms2);

    // -------------------------------------------------------------------------
    // ISR callback
    // -------------------------------------------------------------------------

    /**
     * @brief Called from the endstop interrupt service routine.
     * @param isTop  true if the top endstop fired, false for the bottom.
     */
    void onEndstopTriggered(bool isTop);

    // -------------------------------------------------------------------------
    // Telemetry accessors
    // -------------------------------------------------------------------------

    /** @brief Current encoder position (mm, relative to home). */
    float getPositionMm();

    /** @brief Actual motor velocity measured from the encoder (mm/s). */
    float getActualVelocityMms();

    /** @brief Last commanded velocity (mm/s). */
    float getCommandedVelocityMms() const;

    /** @brief Actual acceleration — not yet implemented, returns 0. */
    float getActualAccelMms2()      const;

    /**
     * @brief Returns true while the motor is actively stepping.
     *
     * Wraps getMotorState(STANDSTILL) from the UstepperS32 library, which
     * returns 1 while stepping and 0 when stopped — inverted from the register
     * name but matching observed behaviour.
     */
    bool isMoving();

private:

    // -------------------------------------------------------------------------
    // Internal types
    // -------------------------------------------------------------------------

    /** @brief Active motion mode, used to dispatch in update(). */
    enum class ProfileMode : uint8_t {
        NONE,
        HOMING,
        TRAPEZOIDAL,
        SEGMENTED,
        JOG,
        LIMIT_BACKOFF,
        MOVING,
    };

    /** @brief One segment in a loaded segmented move. */
    struct Segment {
        enum class Type : uint8_t { MOVE, DWELL };
        Type     type;     ///< Segment type — MOVE executes a displacement, DWELL holds position
        float    distMm;   ///< (MOVE only) Signed displacement (mm, +ve = up)
        float    speedMms; ///< (MOVE only) Speed (mm/s)
        float    accelMms2;///< (MOVE only) Acceleration (mm/s²)
        uint32_t dwellMs;  ///< (DWELL only) Hold time (ms)
    };

    /** @brief Snapshot saved by pause() and consumed by resume(). */
    struct PauseSnapshot {
        ProfileMode mode;
        RunPhase    phase;
        int         currentDip;
        uint8_t     segIndex;
    };

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    StateMachine& _sm;
    UstepperS32   _stepper;

    ProfileMode _mode;
    float       _softLimitMinMm;
    float       _softLimitMaxMm;
    float       _commandedVelocityMms;

    // Trapezoidal profile parameters
    float    _dipSpeedMms;
    float    _withdrawSpeedMms;
    float    _accelMms2;
    float    _depthMm;
    uint32_t _dwellBottomMs;
    uint32_t _dwellTopMs;
    int      _nDips;
    int      _currentDip;

    // Segmented move state
    Segment  _segments[MOVE_SEG_BUFFER_SIZE];
    uint8_t  _segCount;
    uint8_t  _segIndex;

    // Jog state
    bool _jogUp;             ///< Direction saved by jog() — used by resume() to restart a paused jog

    // Move tracking
    float    _targetMm;        ///< Absolute target for the current move (mm)
    float    _moveStartMm;     ///< Position recorded at the start of each move (mm)
    float    _profileStartMm;  ///< Position when runProfile() was called (mm)
    float    _movingSpeedMms;  ///< Speed saved by moveByMm() — used by resume() to restart a paused CMD MOVE (mm/s)
    float    _movingAccelMms2; ///< Accel saved by moveByMm() — used by resume() to restart a paused CMD MOVE (mm/s²)
    uint32_t _moveStartMs;     ///< millis() recorded at the start of each move — used by isMoveComplete() settle guard

    // Dwell state
    uint32_t _dwellStartMs;
    uint32_t _dwellDurationMs;
    bool     _inDwell;

    // Homing state
    bool _homingBackoffActive;

    // Limit-switch backoff state (set from ISR, cleared in update())
    volatile bool _limitTriggered;    ///< New limit event pending
    volatile bool _limitIsTop;        ///< Which endstop fired
    bool          _limitBackoffActive;
    uint32_t      _limitBackoffStart;

    // Pause state
    PauseSnapshot _pauseSnapshot;
    bool          _paused;

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** @brief Unit conversion: mm → degrees of motor rotation. */
    float mmToDeg(float mm) const;

    /** @brief Set stepper max velocity and acceleration from current _accelMms2. */
    void  setSpeed(float speedMms);

    /** @brief Encoder-based position (mm, relative to home). */
    float positionMm();

    /** @brief Encoder-based actual velocity (mm/s). */
    float actualVelocityMms();

    /**
     * @brief Command an absolute move with soft-limit check.
     *        Sets _targetMm, _moveStartMm, and calls stepper.moveToAngle().
     */
    void startMoveToMm(float targetMm, float speedMms);

    /** @brief Begin a timed dwell.  isDwellComplete() checks expiry. */
    void startDwell(uint32_t ms);

    /** @brief Returns true once the dwell timer has elapsed. */
    bool isDwellComplete() const;

    /**
     * @brief Returns true when the motor has stopped at or near the target.
     *
     * Guards against false completion from: brief STANDSTILL glitches before
     * the motor has covered half the commanded distance, and limit-switch stops
     * that leave the motor far from the intended target.
     */
    bool isMoveComplete();

    /**
     * @brief Checks targetMm against the soft limits.
     *        Calls estop() and transitions to ERROR if exceeded.
     * @return true if within limits, false if the move was rejected.
     */
    bool checkSoftLimit(float targetMm);

    // Mode-specific update handlers (called from update())
    void updateHoming();
    void updateTrapezoidal();
    void updateSegmented();
    void updateLimitBackoff();
    void updateMove();

    /**
     * @brief Start executing _segments[_segIndex].
     *        Dispatches to startMoveToMm() for MOVE segments and startDwell()
     *        for DWELL segments.  Called by runLoadedMove(), updateSegmented(),
     *        and resume().
     */
    void startCurrentSegment();
};
