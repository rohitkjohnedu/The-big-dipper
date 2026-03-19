#include "motion_controller.h"
#include "trace.h"

// =============================================================================
// Unit conversion helpers
// =============================================================================

/** @brief Convert linear position (mm) to motor angle (degrees). */
float MotionController::mmToDeg(float mm) const {
    return mm * (360.0f / LEADSCREW_MM_PER_REV);
}

/** @brief Return current encoder position in mm relative to home. */
float MotionController::positionMm() {
    return _stepper.angleMoved() * (LEADSCREW_MM_PER_REV / 360.0f);
}

/** @brief Return actual motor velocity in mm/s from the encoder RPM. */
float MotionController::actualVelocityMms() {
    return _stepper.encoder.getRPM() * (LEADSCREW_MM_PER_REV / 60.0f);
}

/**
 * @brief Apply speed and acceleration to the stepper driver.
 *
 * The UstepperS32 library accepts velocity in deg/s, so all mm/s values
 * are converted via mmToDeg().  Acceleration and deceleration are set to
 * the same value (_accelMms2) to produce symmetric trapezoidal ramps.
 */
void MotionController::setSpeed(float speedMms) {
    _stepper.setMaxVelocity(    mmToDeg(speedMms)   );
    _stepper.setMaxAcceleration(mmToDeg(_accelMms2) );
    _stepper.setMaxDeceleration(mmToDeg(_accelMms2) );
}

// =============================================================================
// Constructor
// =============================================================================

MotionController::MotionController(StateMachine& sm)
    : _sm                 (sm)
    , _mode               (ProfileMode::NONE)
    , _softLimitMinMm     (SOFT_LIMIT_MIN_MM)
    , _softLimitMaxMm     (SOFT_LIMIT_MAX_MM)
    , _commandedVelocityMms(0.0f)
    , _dipSpeedMms        (DEFAULT_DIP_SPEED_MM_S)
    , _withdrawSpeedMms   (DEFAULT_WITHDRAW_SPEED_MM_S)
    , _accelMms2          (DEFAULT_ACCEL_MM_S2)
    , _depthMm            (DEFAULT_DIP_DEPTH_MM)
    , _dwellBottomMs      (DEFAULT_DWELL_BOTTOM_MS)
    , _dwellTopMs         (DEFAULT_DWELL_TOP_MS)
    , _nDips              (DEFAULT_N_DIPS)
    , _currentDip         (1)
    , _segCount           (0)
    , _segIndex           (0)
    , _segNDips           (1)
    , _segDwellBottomMs   (0)
    , _segDwellTopMs      (0)
    , _segCurrentDip      (1)
    , _targetMm           (0.0f)
    , _moveStartMm        (0.0f)
    , _profileStartMm     (0.0f)
    , _dwellStartMs       (0)
    , _dwellDurationMs    (0)
    , _inDwell            (false)
    , _homingBackoffActive(false)
    , _limitTriggered     (false)
    , _limitIsTop         (false)
    , _limitBackoffActive (false)
    , _limitBackoffStart  (0)
    , _paused             (false)
{}

// =============================================================================
// begin()  — call once from setup()
// =============================================================================

void MotionController::begin() {
    // Parameters: mode, steps/rev, pTerm, iTerm, dTerm,
    //             dropinStepSize, setHome, invert, runCurrent%, holdCurrent%
    // invert = 0: verify on hardware — set to 1 if motor direction is reversed.
    _stepper.setup(NORMAL, MOTOR_STEPS_PER_REV,
                   10.0f, 0.0f, 0.0f,
                   16, false, 0, 50, 30);

    // Set TPWMTHRS to the crossover speed between StealthChop (quiet, below
    // threshold) and SpreadCycle (more torque, above threshold). See config.h.
    _stepper.driver.writeRegister(TPWMTHRS,   STEALTH_TPWMTHRS);

    // Delay before hold current activates — prevents audible click on stop.
    _stepper.driver.writeRegister(TPOWERDOWN, STEALTH_TPOWERDOWN);
}

// =============================================================================
// update()  — call every loop()
// =============================================================================

void MotionController::update() {
    switch (_mode) {
        case ProfileMode::HOMING:        updateHoming();        break;
        case ProfileMode::TRAPEZOIDAL:   updateTrapezoidal();   break;
        case ProfileMode::SEGMENTED:     updateSegmented();     break;
        case ProfileMode::JOG:                                  break;  // library handles continuous motion
        case ProfileMode::MOVING:        updateMove();          break;
        case ProfileMode::LIMIT_BACKOFF: updateLimitBackoff();  break;
        case ProfileMode::NONE:                                 break;
    }
}

// =============================================================================
// Public commands
// =============================================================================

void MotionController::executeHome() {
    _mode                 = ProfileMode::HOMING;
    _homingBackoffActive  = false;
    _commandedVelocityMms = HOMING_SPEED_MM_S;

    // Use positioning mode (moveAngle) rather than velocity mode (runContinous)
    // so StealthChop remains active.  Command more than max travel upward —
    // the top-endstop ISR will hard-stop the motor.
    _stepper.setMaxVelocity(    mmToDeg(HOMING_SPEED_MM_S)  );
    _stepper.setMaxAcceleration(mmToDeg(HOMING_ACC_MM_S2)   );
    _stepper.setMaxDeceleration(mmToDeg(HOMING_ACC_MM_S2)   );
    _stepper.moveAngle(mmToDeg(TRAVEL_MAX_MM));
}

void MotionController::setSoftLimits(float minMm, float maxMm) {
    _softLimitMinMm = minMm;
    _softLimitMaxMm = maxMm;
}

void MotionController::stop() {
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _sm.toReady();
    _stepper.stop(SOFT);
}

void MotionController::estop() {
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _paused               = false;
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
        _currentDip      = _pauseSnapshot.currentDip;
        RunPhase ph      = _pauseSnapshot.phase;
        if      (ph == RunPhase::DESCENDING)   startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
        else if (ph == RunPhase::ASCENDING)    startMoveToMm(_profileStartMm,            _withdrawSpeedMms);
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

void MotionController::jog(bool up, float speedMms) {
    _mode                 = ProfileMode::JOG;
    _commandedVelocityMms = speedMms;
    _accelMms2            = DEFAULT_ACCEL_MM_S2;
    setSpeed(speedMms);
    _stepper.runContinous(up ? CW : CCW);   // CW = up, CCW = down
}

void MotionController::moveByMm(float deltaMm, float speedMms, float accelMms2) {
    float saved = _accelMms2;
    _accelMms2  = accelMms2;
    _mode       = ProfileMode::MOVING;
    startMoveToMm(positionMm() + deltaMm, speedMms);
    _accelMms2  = saved;    // restore so normal profile moves are unaffected
}

void MotionController::runProfile(float dipSpeedMms,    float withdrawSpeedMms,
                                   float accelMms2,      float depthMm,
                                   int   dwellBottomMs,  int   dwellTopMs,
                                   int   nDips) {
    _dipSpeedMms      = dipSpeedMms;
    _withdrawSpeedMms = withdrawSpeedMms;
    _accelMms2        = accelMms2;
    _depthMm          = depthMm;
    _dwellBottomMs    = (uint32_t)dwellBottomMs;
    _dwellTopMs       = (uint32_t)dwellTopMs;
    _nDips            = nDips;
    _currentDip       = 1;
    _profileStartMm   = positionMm();   // dip depth is relative to current position
    _mode             = ProfileMode::TRAPEZOIDAL;
    _sm.toRunning();
    _sm.setPhase(RunPhase::DESCENDING);
    TR2F("runProfile depth=", _depthMm, " dipSpd=", _dipSpeedMms);
    startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
}

void MotionController::beginSegmentedMove(uint8_t nDips,
                                           uint16_t dwellBottomMs,
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
    // During homing the top endstop is expected — zero the encoder and back off.
    if (_mode == ProfileMode::HOMING && isTop && !_homingBackoffActive) {
        _commandedVelocityMms = 0.0f;
        _stepper.stop(HARD);
        _stepper.encoder.setHome();   // zero encoder at the top endstop
        _homingBackoffActive = true;
        startMoveToMm(-HOMING_BACKOFF_MM, HOMING_SPEED_MM_S);
        return;
    }

    // Ignore spurious triggers when idle or already backing off.
    if (_mode == ProfileMode::NONE || _mode == ProfileMode::LIMIT_BACKOFF) return;

    // Unexpected endstop hit during motion — hard-stop and schedule a backoff.
    _stepper.stop(HARD);
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _paused               = false;
    _limitTriggered       = true;
    _limitIsTop           = isTop;
    _mode                 = ProfileMode::LIMIT_BACKOFF;
}

// =============================================================================
// Telemetry accessors
// =============================================================================

float MotionController::getPositionMm()           { return positionMm();          }
float MotionController::getActualVelocityMms()    { return actualVelocityMms();   }
float MotionController::getCommandedVelocityMms() const { return _commandedVelocityMms; }
float MotionController::getActualAccelMms2()      const { return 0.0f;            }   // not yet implemented

bool MotionController::isStandstill() {
    return _stepper.getMotorState(STANDSTILL);
}

// =============================================================================
// Private helpers
// =============================================================================

void MotionController::startMoveToMm(float targetMm, float speedMms) {
    if (!checkSoftLimit(targetMm)) return;     // estop + ERROR state set inside
    _moveStartMm          = positionMm();
    _targetMm             = targetMm;
    _commandedVelocityMms = speedMms;
    TR2F("startMoveToMm pos=", _moveStartMm, " target=", targetMm);
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

    // getMotorState(STANDSTILL) returns 1 while the motor is actively stepping
    // and 0 when it has stopped.  So "complete" = not stepping any more.
    if (_stepper.getMotorState(STANDSTILL)) return false;

    float pos      = positionMm();
    float travelMm = fabsf(_targetMm  - _moveStartMm);
    float movedMm  = fabsf(pos        - _moveStartMm);
    float errorMm  = fabsf(pos        - _targetMm);

    // Reject a spurious early stop before the motor has covered half the distance.
    if (travelMm > 0.5f && movedMm < travelMm * 0.5f) return false;

    // Reject if the motor stopped far from the target (e.g. unexpected limit hit).
    if (travelMm > 0.5f && errorMm > 3.0f)             return false;

    TR2F("isMoveComplete pos=", pos, " target=", _targetMm);
    return true;
}

bool MotionController::checkSoftLimit(float targetMm) {
    if (targetMm < _softLimitMinMm || targetMm > _softLimitMaxMm) {
        Serial.print("ERR SOFT_LIMIT_EXCEEDED target=");
        Serial.print(targetMm, 2);
        Serial.print("mm limits=[");
        Serial.print(_softLimitMinMm, 2);
        Serial.print(", ");
        Serial.print(_softLimitMaxMm, 2);
        Serial.println("]");
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
    if (!_homingBackoffActive) return;   // waiting for the endstop ISR to fire
    if (isMoveComplete()) {
        _mode                 = ProfileMode::NONE;
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
            startMoveToMm(_profileStartMm, _withdrawSpeedMms);
        } else if (phase == RunPhase::DWELL_TOP) {
            TR("dwell_top done -> DESCENDING");
            _sm.setPhase(RunPhase::DESCENDING);
            startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
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
            _mode                 = ProfileMode::NONE;
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
            _mode                 = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.setPhase(RunPhase::NONE);
            _sm.toReady();
        }
    }
}

// =============================================================================
// updateLimitBackoff
// =============================================================================

void MotionController::updateLimitBackoff() {
    uint32_t now = millis();

    if (_limitTriggered) {
        _limitTriggered = false;
        Serial.print(_limitIsTop ? "TOP" : "BOTTOM");
        Serial.println(" LIMIT switch triggered");

        // Back off away from the triggered endstop.
        // positionMm() = 0 at home (top), positive = up, negative = down.
        //   Top triggered    → move down (subtract backoff distance)
        //   Bottom triggered → move up   (add    backoff distance)
        float target = _limitIsTop
            ? positionMm() - LIMIT_BACKOFF_MM
            : positionMm() + LIMIT_BACKOFF_MM;
        _limitBackoffActive = true;
        _limitBackoffStart  = now;
        startMoveToMm(target, LIMIT_BACKOFF_SPEED_MM_S);
        return;
    }

    if (_limitBackoffActive) {
        uint32_t elapsed = now - _limitBackoffStart;
        if (elapsed < 200) return;   // give the motor time to start moving
        if (isMoveComplete() || elapsed >= 5000) {
            _limitBackoffActive   = false;
            _mode                 = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.toReady();
        }
    }
}

// =============================================================================
// updateMove  (CMD MOVE / single relative move)
// =============================================================================

void MotionController::updateMove() {
    // Diagnostics manages its own completion via isStandstill().
    // This handler only fires for CMD MOVE (state == RUNNING).
    if (_sm.getState() != SystemState::RUNNING) return;

    if (isMoveComplete()) {
        _mode                 = ProfileMode::NONE;
        _commandedVelocityMms = 0.0f;
        _sm.toReady();
        Serial.println("CMD:MOVE:DONE");
    }
}
