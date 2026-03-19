#pragma once
#include <Arduino.h>
#include <UstepperS32.h>
#include "config.h"
#include "state_machine.h"

// =============================================================================
// MotionController
//
// Owns the UstepperS32 instance. Executes all motion commands.
// Call begin() from setup() and update() from every loop().
// Non-blocking — all timing via millis(). Never calls delay().
//
// Coordinate system:
//   Home (top endstop) = 0 mm.  Positive mm = Upward.
//   CW = up,  CCW = down.  Swap if wiring is reversed.
// =============================================================================

class MotionController {
public:
    explicit MotionController(StateMachine& sm);

    void begin();    // call once from setup()
    void update();   // call every loop()

    // Commands — called by CommandParser
    void executeHome();
    void setSoftLimits(float minMm, float maxMm);
    void setDebug(bool on);
    void stop();
    void estop();
    void pause();
    void resume();
    void jog(bool up, float speedMms);

    void runProfile(float dipSpeedMms, float withdrawSpeedMms, float accelMms2,
                    float depthMm, int dwellBottomMs, int dwellTopMs, int nDips);

    void beginSegmentedMove(uint8_t nDips, uint16_t dwellBottomMs, uint16_t dwellTopMs);
    bool addSegment(float distMm, float speedMms);   // false if buffer full
    void runLoadedMove();

    // Single relative move — used by Diagnostics
    void moveByMm(float deltaMm, float speedMms, float accelMms2);
    bool isMoveDone();

    // ISR callback — wire to endstop interrupt handlers in .ino
    void onEndstopTriggered(bool isTop);

    // Telemetry accessors
    float getPositionMm();
    float getActualVelocityMms();
    float getCommandedVelocityMms() const;
    float getActualAccelMms2()      const;

    bool isStandstill();

private:
    enum class ProfileMode : uint8_t { NONE, HOMING, TRAPEZOIDAL, SEGMENTED, JOG, LIMIT_BACKOFF, MOVING };

    struct Segment {
        float distMm;    // signed: +ve = down
        float speedMms;
    };

    struct PauseSnapshot {
        ProfileMode mode;
        RunPhase    phase;
        int         currentDip;
        uint8_t     segIndex;
        int         segDip;
    };

    StateMachine& _sm;
    UstepperS32   _stepper;

    ProfileMode _mode;
    float _softLimitMinMm;
    float _softLimitMaxMm;
    bool  _debugOn;
    float _commandedVelocityMms;

    // Trapezoidal profile
    float    _dipSpeedMms;
    float    _withdrawSpeedMms;
    float    _accelMms2;
    float    _depthMm;
    uint32_t _dwellBottomMs;
    uint32_t _dwellTopMs;
    int      _nDips;
    int      _currentDip;

    // Segmented move
    Segment  _segments[MOVE_SEG_BUFFER_SIZE];
    uint8_t  _segCount;
    uint8_t  _segIndex;
    uint8_t  _segNDips;
    uint16_t _segDwellBottomMs;
    uint16_t _segDwellTopMs;
    int      _segCurrentDip;

    float    _targetMm;
    float    _moveStartMm;     // position recorded at the start of each move
    float    _profileStartMm;  // position when runProfile() was called — dip/withdraw relative to this

    // Dwell
    uint32_t _dwellStartMs;
    uint32_t _dwellDurationMs;
    bool     _inDwell;

    // Single move (moveByMm)
    uint32_t _movingStartMs;

    // Homing
    bool _homingBackoffActive;

    // Limit switch backoff — set from ISR, cleared in update()
    volatile bool _limitTriggered;   // new limit event pending
    volatile bool _limitIsTop;       // which endstop fired
    bool          _limitBackoffActive;
    uint32_t      _limitBackoffStart;

    // Pause
    PauseSnapshot _pauseSnapshot;
    bool          _paused;

    // Helpers
    void  startMoveToMm(float targetMm, float speedMms);
    void  startDwell(uint32_t ms);
    bool  isDwellComplete() const;
    bool  isMoveComplete();
    bool  checkSoftLimit(float targetMm);
    float positionMm();
    float actualVelocityMms();
    float mmToDeg(float mm) const;
    void  setSpeed(float speedMms);

    void updateHoming();
    void updateTrapezoidal();
    void updateSegmented();
    void updateLimitBackoff();
    void updateMove();


};
