#pragma once

#ifdef ARDUINO
#  include <Arduino.h>
#else
#  include <cstdint>
#endif

#include "config.h"
#include "state_machine.h"

#ifdef ARDUINO
#  include <uStepperS32.h>
#endif

// =============================================================================
// MotionController
//
// Owns the uStepperS32 instance. Executes all motion commands.
// Called every loop() via update(). Never blocks — all timing via millis().
//
// Unit convention (matches architecture brief):
//   External interface  : mm, mm/s, mm/s²
//   uStepperS32 library : steps/s (velocity), degrees (position)
//   Conversion          : see mmToDeg() below
//
// VERIFY_API comments mark calls that must be checked against the installed
// uStepperS32 library version before first flash.
// =============================================================================

class MotionController {
public:
    explicit MotionController(StateMachine& sm);

    // Called from setup() and loop() in dip_coater_firmware.ino
    void begin();
    void update();

    // -------------------------------------------------------------------------
    // Commands — called by CommandParser
    // -------------------------------------------------------------------------
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
    bool addSegment(float distMm, float speedMms);   // returns false if buffer full
    void runLoadedMove();

    // -------------------------------------------------------------------------
    // ISR callback — called from endstop interrupt handlers in .ino
    // -------------------------------------------------------------------------
    void onEndstopTriggered(bool isTop);

    // -------------------------------------------------------------------------
    // Telemetry interface — called by Telemetry every loop()
    // -------------------------------------------------------------------------
    float getPositionMm();
    float getActualVelocityMms();
    float getCommandedVelocityMms() const;
    float getActualAccelMms2()     const;

private:
    // -------------------------------------------------------------------------
    // Profile execution mode
    // -------------------------------------------------------------------------
    enum class ProfileMode : uint8_t {
        NONE,
        HOMING,
        TRAPEZOIDAL,
        SEGMENTED,
        JOG
    };

    // -------------------------------------------------------------------------
    // Segment buffer entry
    // -------------------------------------------------------------------------
    struct Segment {
        float distMm;    // displacement for this segment (signed: +ve = down)
        float speedMms;  // speed magnitude
    };

    // -------------------------------------------------------------------------
    // Snapshot saved on PAUSE, restored on RESUME
    // -------------------------------------------------------------------------
    struct PauseSnapshot {
        ProfileMode mode;
        RunPhase    phase;
        int         currentDip;   // which dip we were on (trapezoidal)
        uint8_t     segIndex;     // which segment we were on (segmented)
        int         segDip;       // which dip we were on (segmented)
    };

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------
    StateMachine& _sm;

#ifdef ARDUINO
    UstepperS32 _stepper;
#endif

    ProfileMode _mode;

    float _softLimitMinMm;
    float _softLimitMaxMm;
    bool  _debugOn;

    float _commandedVelocityMms;  // tracked here, read by Telemetry

    // --- Trapezoidal profile params ------------------------------------------
    float    _dipSpeedMms;
    float    _withdrawSpeedMms;
    float    _accelMms2;
    float    _depthMm;
    uint32_t _dwellBottomMs;
    uint32_t _dwellTopMs;
    int      _nDips;
    int      _currentDip;         // 1-indexed, current dip number

    // --- Segmented move params -----------------------------------------------
    Segment  _segments[MOVE_SEG_BUFFER_SIZE];
    uint8_t  _segCount;
    uint8_t  _segIndex;           // next segment to execute
    uint8_t  _segNDips;
    uint16_t _segDwellBottomMs;
    uint16_t _segDwellTopMs;
    int      _segCurrentDip;      // 1-indexed

    // Current absolute target used by both modes (for soft-limit check)
    float _targetMm;

    // --- Dwell timing --------------------------------------------------------
    uint32_t _dwellStartMs;
    uint32_t _dwellDurationMs;
    bool     _inDwell;

    // --- Homing state --------------------------------------------------------
    bool _homingBackoffActive;    // true during the backoff move after endstop hit

    // --- Pause state ---------------------------------------------------------
    PauseSnapshot _pauseSnapshot;
    bool          _paused;

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    // Start an absolute move. Updates _commandedVelocityMms and _targetMm.
    void startMoveToMm(float targetMm, float speedMms);

    // Dwell helpers
    void startDwell(uint32_t ms);
    bool isDwellComplete() const;

    // True when the library reports the motor has stopped (velocity ≈ 0)
    bool isMoveComplete();

    // Check targetMm against soft limits. If violated, raises ERROR and
    // returns false. Caller must abort the command if false.
    bool checkSoftLimit(float targetMm);

    // Per-mode update callbacks
    void updateHoming();
    void updateTrapezoidal();
    void updateSegmented();

#ifdef ARDUINO
    // Unit conversion
    float positionMm();         // actual encoder position → mm
    float actualVelocityMms(); // encoder RPM → mm/s
    float mmToDeg(float mm)   const;  // mm → degrees of shaft rotation
#endif
};
