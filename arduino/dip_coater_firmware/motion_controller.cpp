#include "motion_controller.h"

// =============================================================================
// Trace macros — enable with #define TRACE in config.h
// Output format: "TR <millis> <message>"
// =============================================================================

#ifdef TRACE
  #define TR(msg) do { \
      Serial.print(F("TR ")); Serial.print(millis()); \
      Serial.print(' '); Serial.println(F(msg)); } while(0)
  #define TRF(msg, val) do { \
      Serial.print(F("TRF ")); Serial.print(millis()); \
      Serial.print(' '); Serial.print(F(msg)); Serial.println(val, 2); } while(0)
  #define TR2F(msg, v1, sep, v2) do { \
      Serial.print(F("TR2F ")); Serial.print(millis()); \
      Serial.print(' '); Serial.print(F(msg)); Serial.print(v1, 2); \
      Serial.print(F(sep)); Serial.println(v2, 2); } while(0)
#else
  #define TR(msg)
  #define TRF(msg, val)
  #define TR2F(msg, v1, sep, v2)
#endif

// =============================================================================
// Unit conversion helpers
// =============================================================================

float MotionController::positionMm() {
    return _stepper.angleMoved() * (LEADSCREW_MM_PER_REV / 360.0f);
}

float MotionController::actualVelocityMms() {
    return _stepper.encoder.getRPM() * (LEADSCREW_MM_PER_REV / 60.0f);
}

float MotionController::mmToDeg(float mm) const {
    return mm * (360.0f / LEADSCREW_MM_PER_REV);
}

// Set velocity and acceleration. Library takes deg/s, not steps/s.
void MotionController::setSpeed(float speedMms) {
    _stepper.setMaxVelocity(mmToDeg(speedMms));
    _stepper.setMaxAcceleration(mmToDeg(_accelMms2));
    _stepper.setMaxDeceleration(mmToDeg(_accelMms2));
}

// =============================================================================
// Constructor
// =============================================================================

MotionController::MotionController(StateMachine& sm)
    : _sm(sm)
    , _mode(ProfileMode::NONE)
    , _softLimitMinMm(SOFT_LIMIT_MIN_MM)
    , _softLimitMaxMm(SOFT_LIMIT_MAX_MM)
    , _debugOn(DEBUG_DEFAULT_ON)
    , _commandedVelocityMms(0.0f)
    , _dipSpeedMms(DEFAULT_DIP_SPEED_MM_S)
    , _withdrawSpeedMms(DEFAULT_WITHDRAW_SPEED_MM_S)
    , _accelMms2(DEFAULT_ACCEL_MM_S2)
    , _depthMm(DEFAULT_DIP_DEPTH_MM)
    , _dwellBottomMs(DEFAULT_DWELL_BOTTOM_MS)
    , _dwellTopMs(DEFAULT_DWELL_TOP_MS)
    , _nDips(DEFAULT_N_DIPS)
    , _currentDip(1)
    , _segCount(0)
    , _segIndex(0)
    , _segNDips(1)
    , _segDwellBottomMs(0)
    , _segDwellTopMs(0)
    , _segCurrentDip(1)
    , _targetMm(0.0f)
    , _dwellStartMs(0)
    , _dwellDurationMs(0)
    , _inDwell(false)
    , _movingStartMs(0)
    , _homingBackoffActive(false)
    , _limitTriggered(false)
    , _limitIsTop(false)
    , _limitBackoffActive(false)
    , _limitBackoffStart(0)
    , _paused(false)
{}

// =============================================================================
// begin() — call from setup()
// =============================================================================

void MotionController::begin() {
    // Parameters: mode, steps/rev, pTerm, iTerm, dTerm,
    //             dropinStepSize, setHome, invert, runCurrent%, holdCurrent%
    // invert=0: verify on hardware — set to 1 if motor direction is reversed
    _stepper.setup(NORMAL, MOTOR_STEPS_PER_REV,
                   10.0f, 0.0f, 0.0f,
                   16, false, 0, 50, 30);
}

// =============================================================================
// update() — call every loop()
// =============================================================================

void MotionController::update() {
    switch (_mode) {
        case ProfileMode::HOMING:      updateHoming();      break;
        case ProfileMode::TRAPEZOIDAL: updateTrapezoidal(); break;
        case ProfileMode::SEGMENTED:   updateSegmented();   break;
        case ProfileMode::JOG:           break;   // library handles continuous motion
        case ProfileMode::MOVING:        break;   // caller resets via stop()
        case ProfileMode::LIMIT_BACKOFF: updateLimitBackoff(); break;
        case ProfileMode::NONE:          break;
    }
}

// =============================================================================
// Commands
// =============================================================================

void MotionController::executeHome() {
    _mode = ProfileMode::HOMING;
    _homingBackoffActive = false;
    _commandedVelocityMms = HOMING_SPEED_MM_S;
    // CCW = UP toward top endstop. Swap to CW if direction is inverted.
    _stepper.setMaxVelocity(mmToDeg(HOMING_SPEED_MM_S));
    _stepper.setMaxAcceleration(mmToDeg(HOMING_ACC_MM_S2));
    _stepper.setMaxDeceleration(mmToDeg(HOMING_ACC_MM_S2));
    _stepper.runContinous(CCW);
}

void MotionController::setSoftLimits(float minMm, float maxMm) {
    _softLimitMinMm = minMm;
    _softLimitMaxMm = maxMm;
}

void MotionController::setDebug(bool on) { _debugOn = on; }

void MotionController::stop() {
    _mode = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell = false;
    _sm.toReady();
    _stepper.stop(SOFT);
}

void MotionController::estop() {
    _mode = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell = false;
    _paused = false;
    _sm.toError(ErrorCode::NONE);
    _stepper.stop(HARD);
}

void MotionController::pause() {
    _pauseSnapshot.mode       = _mode;
    _pauseSnapshot.phase      = _sm.getPhase();
    _pauseSnapshot.currentDip = _currentDip;
    _pauseSnapshot.segIndex   = _segIndex;
    _pauseSnapshot.segDip     = _segCurrentDip;
    _paused  = true;
    _mode    = ProfileMode::NONE;
    _inDwell = false;
    _sm.toPaused();
    _stepper.stop(SOFT);
}

void MotionController::resume() {
    if (!_paused) return;
    _paused = false;
    _mode   = _pauseSnapshot.mode;
    _sm.toRunning();
    _sm.setPhase(_pauseSnapshot.phase);

    if (_mode == ProfileMode::TRAPEZOIDAL) {
        _currentDip = _pauseSnapshot.currentDip;
        RunPhase ph = _pauseSnapshot.phase;
        if      (ph == RunPhase::DESCENDING)   startMoveToMm(_depthMm, _dipSpeedMms);
        else if (ph == RunPhase::ASCENDING)    startMoveToMm(0.0f, _withdrawSpeedMms);
        else if (ph == RunPhase::DWELL_BOTTOM) startDwell(_dwellBottomMs);
        else if (ph == RunPhase::DWELL_TOP)    startDwell(_dwellTopMs);

    } else if (_mode == ProfileMode::SEGMENTED) {
        _segIndex      = _pauseSnapshot.segIndex;
        _segCurrentDip = _pauseSnapshot.segDip;
        RunPhase ph    = _pauseSnapshot.phase;
        if (ph == RunPhase::DESCENDING || ph == RunPhase::ASCENDING) {
            if (_segIndex < _segCount) {
                float target = getPositionMm() + _segments[_segIndex].distMm;
                startMoveToMm(target, _segments[_segIndex].speedMms);
            }
        } else if (ph == RunPhase::DWELL_BOTTOM) startDwell(_segDwellBottomMs);
        else if   (ph == RunPhase::DWELL_TOP)    startDwell(_segDwellTopMs);
    }
}

void MotionController::moveByMm(float deltaMm, float speedMms, float accelMms2) {
    float saved = _accelMms2;
    _accelMms2 = accelMms2;
    _mode = ProfileMode::MOVING;
    _movingStartMs = millis();
    startMoveToMm(positionMm() + deltaMm, speedMms);
    _accelMms2 = saved;          // restore for normal profile moves
}

bool MotionController::isMoveDone() {
    return isMoveComplete();
}

void MotionController::jog(bool up, float speedMms) {
    _mode = ProfileMode::JOG;
    _commandedVelocityMms = speedMms;
    _accelMms2 = DEFAULT_ACCEL_MM_S2;
    setSpeed(speedMms);
    _stepper.runContinous(up ? CCW : CW);   // CCW=up, CW=down
}

void MotionController::runProfile(float dipSpeedMms, float withdrawSpeedMms,
                                   float accelMms2, float depthMm,
                                   int dwellBottomMs, int dwellTopMs, int nDips) {
    _dipSpeedMms      = dipSpeedMms;
    _withdrawSpeedMms = withdrawSpeedMms;
    _accelMms2        = accelMms2;
    _depthMm          = depthMm;
    _dwellBottomMs    = (uint32_t)dwellBottomMs;
    _dwellTopMs       = (uint32_t)dwellTopMs;
    _nDips            = nDips;
    _currentDip       = 1;
    _mode             = ProfileMode::TRAPEZOIDAL;
    _sm.toRunning();
    _sm.setPhase(RunPhase::DESCENDING);
    TR2F("runProfile depth=", _depthMm, " dipSpd=", _dipSpeedMms);
    startMoveToMm(_depthMm, _dipSpeedMms);
}

void MotionController::beginSegmentedMove(uint8_t nDips, uint16_t dwellBottomMs,
                                           uint16_t dwellTopMs) {
    _segNDips         = nDips;
    _segDwellBottomMs = dwellBottomMs;
    _segDwellTopMs    = dwellTopMs;
    _segCount         = 0;
    _segIndex         = 0;
    _segCurrentDip    = 1;
}

bool MotionController::addSegment(float distMm, float speedMms) {
    if (_segCount >= MOVE_SEG_BUFFER_SIZE) return false;
    _segments[_segCount].distMm   = distMm;
    _segments[_segCount].speedMms = speedMms;
    _segCount++;
    return true;
}

void MotionController::runLoadedMove() {
    if (_segCount == 0) return;
    _segIndex      = 0;
    _segCurrentDip = 1;
    _mode          = ProfileMode::SEGMENTED;
    _sm.toRunning();
    _sm.setPhase(RunPhase::DESCENDING);
    float target = getPositionMm() + _segments[0].distMm;
    startMoveToMm(target, _segments[0].speedMms);
}

// =============================================================================
// ISR callback
// =============================================================================

void MotionController::onEndstopTriggered(bool isTop) {
    if (_mode == ProfileMode::HOMING && isTop && !_homingBackoffActive) {
        _commandedVelocityMms = 0.0f;
        _stepper.stop(HARD);
        _stepper.encoder.setHome();   // zero encoder at top endstop
        _homingBackoffActive = true;
        startMoveToMm(HOMING_BACKOFF_MM, HOMING_SPEED_MM_S);
        return;
    }
    // Only respond if motor is actually moving; ignore when idle or already backing off
    if (_mode == ProfileMode::NONE || _mode == ProfileMode::LIMIT_BACKOFF) return;
    // Limit switch hit during motion — hard stop, back off in update()
    _stepper.stop(HARD);
    _commandedVelocityMms = 0.0f;
    _inDwell  = false;
    _paused   = false;
    _limitTriggered = true;
    _limitIsTop     = isTop;
    _mode = ProfileMode::LIMIT_BACKOFF;
}

// =============================================================================
// Telemetry
// =============================================================================

float MotionController::getPositionMm()          { return positionMm(); }
float MotionController::getActualVelocityMms()   { return actualVelocityMms(); }
float MotionController::getCommandedVelocityMms() const { return _commandedVelocityMms; }
float MotionController::getActualAccelMms2()      const { return 0.0f; }

// =============================================================================
// Private helpers
// =============================================================================

void MotionController::startMoveToMm(float targetMm, float speedMms) {
    TR2F("startMoveToMm pos=", positionMm(), " target=", targetMm);
    _targetMm             = targetMm;
    _commandedVelocityMms = speedMms;
    setSpeed(speedMms);
    _stepper.moveToAngle(mmToDeg(targetMm));
}

void MotionController::startDwell(uint32_t ms) {
    TRF("startDwell ms=", (float)ms);
    _inDwell          = true;
    _dwellDurationMs  = ms;
    _dwellStartMs     = millis();
    _commandedVelocityMms = 0.0f;
}

bool MotionController::isDwellComplete() const {
    if (!_inDwell) return false;
    return (millis() - _dwellStartMs) >= _dwellDurationMs;
}

bool MotionController::isMoveComplete() {
    if (_inDwell) return false;
    bool done = _stepper.getMotorState(STANDSTILL);
    if (done) { TR2F("isMoveComplete pos=", positionMm(), " target=", _targetMm); }
    return done;
}

bool MotionController::checkSoftLimit(float targetMm) {
    if (targetMm < _softLimitMinMm || targetMm > _softLimitMaxMm) {
        estop();
        _sm.toError(ErrorCode::SOFT_LIMIT_EXCEEDED);
        return false;
    }
    return true;
}

// =============================================================================
// updateHoming
// =============================================================================

void MotionController::updateHoming() {
    if (!_homingBackoffActive) return;   // waiting for ISR to fire
    if (isMoveComplete()) {
        _mode = ProfileMode::NONE;
        _commandedVelocityMms = 0.0f;
        _sm.toReady();
    }
}

// =============================================================================
// updateTrapezoidal
// =============================================================================

void MotionController::updateTrapezoidal() {
    RunPhase phase = _sm.getPhase();

    if (_inDwell) {
        if (!isDwellComplete()) return;
        _inDwell = false;
        if (phase == RunPhase::DWELL_BOTTOM) {
            TR("dwell_bottom done -> ASCENDING");
            _sm.setPhase(RunPhase::ASCENDING);
            startMoveToMm(0.0f, _withdrawSpeedMms);
        } else if (phase == RunPhase::DWELL_TOP) {
            TR("dwell_top done -> DESCENDING");
            _sm.setPhase(RunPhase::DESCENDING);
            startMoveToMm(_depthMm, _dipSpeedMms);
        }
        return;
    }

    if (!isMoveComplete()) return;

    if (phase == RunPhase::DESCENDING) {
        TR("DESCENDING done -> DWELL_BOTTOM");
        _sm.setPhase(RunPhase::DWELL_BOTTOM);
        startDwell(_dwellBottomMs);
    } else if (phase == RunPhase::ASCENDING) {
        if (_currentDip < _nDips) {
            _currentDip++;
            TR("ASCENDING done -> DWELL_TOP");
            _sm.setPhase(RunPhase::DWELL_TOP);
            startDwell(_dwellTopMs);
        } else {
            TR("ASCENDING done -> profile complete");
            _mode = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.setPhase(RunPhase::NONE);
            _sm.toReady();
        }
    }
}

// =============================================================================
// updateSegmented
// =============================================================================

void MotionController::updateSegmented() {
    RunPhase phase = _sm.getPhase();

    if (_inDwell) {
        if (!isDwellComplete()) return;
        _inDwell = false;
        if (phase == RunPhase::DWELL_BOTTOM) {
            _sm.setPhase(RunPhase::ASCENDING);
            if (_segIndex < _segCount) {
                float target = getPositionMm() + _segments[_segIndex].distMm;
                startMoveToMm(target, _segments[_segIndex].speedMms);
            }
        } else if (phase == RunPhase::DWELL_TOP) {
            _segIndex = 0;
            _sm.setPhase(RunPhase::DESCENDING);
            float target = getPositionMm() + _segments[0].distMm;
            startMoveToMm(target, _segments[0].speedMms);
        }
        return;
    }

    if (!isMoveComplete()) return;

    _segIndex++;
    if (_segIndex < _segCount) {
        float target = getPositionMm() + _segments[_segIndex].distMm;
        startMoveToMm(target, _segments[_segIndex].speedMms);
        return;
    }

    if (phase == RunPhase::DESCENDING) {
        _sm.setPhase(RunPhase::DWELL_BOTTOM);
        startDwell(_segDwellBottomMs);
        _segIndex = 0;
    } else if (phase == RunPhase::ASCENDING) {
        if (_segCurrentDip < _segNDips) {
            _segCurrentDip++;
            _sm.setPhase(RunPhase::DWELL_TOP);
            startDwell(_segDwellTopMs);
        } else {
            _mode = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.setPhase(RunPhase::NONE);
            _sm.toReady();
        }
    }
}

// =============================================================================
// updateLimitBackoff — called from update() while in LIMIT_BACKOFF mode
// =============================================================================

void MotionController::updateLimitBackoff() {
    uint32_t now = millis();

    if (_limitTriggered) {
        _limitTriggered = false;
        Serial.print(_limitIsTop ? "TOP" : "BOTTOM");
        Serial.println(" LIMIT switch triggered");
        // Back off away from the triggered endstop.
        // positionMm() = 0 at home (top), positive going down.
        // Top triggered: move down  → add backoff
        // Bottom triggered: move up → subtract backoff
        float target = _limitIsTop
            ? positionMm() + LIMIT_BACKOFF_MM
            : positionMm() - LIMIT_BACKOFF_MM;
        _limitBackoffActive = true;
        _limitBackoffStart  = now;
        startMoveToMm(target, LIMIT_BACKOFF_SPEED_MM_S);
        return;
    }

    if (_limitBackoffActive) {
        uint32_t elapsed = now - _limitBackoffStart;
        if (elapsed < 200) return;   // give motor time to start moving before polling
        if (isMoveComplete() || elapsed >= 5000) {
            _limitBackoffActive   = false;
            _mode                 = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.toReady();
        }
    }
}
