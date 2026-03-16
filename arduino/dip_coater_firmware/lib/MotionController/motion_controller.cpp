#include "motion_controller.h"

#ifdef ARDUINO
#  include <Arduino.h>
#endif

// =============================================================================
// Helpers — unit conversion
// =============================================================================

#ifdef ARDUINO

float MotionController::positionMm() {
    // angleMoved() returns degrees from the reference set by encoder.setHome().
    // One full revolution = LEADSCREW_MM_PER_REV mm of linear travel.
    return _stepper.angleMoved() * (LEADSCREW_MM_PER_REV / 360.0f);
}

float MotionController::actualVelocityMms() {
    // getRPM() returns shaft speed in RPM from the TLE5012B encoder.
    return _stepper.encoder.getRPM() * (LEADSCREW_MM_PER_REV / 60.0f);
}

float MotionController::mmToDeg(float mm) const {
    return mm * (360.0f / LEADSCREW_MM_PER_REV);
}

#endif  // ARDUINO

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
    , _homingBackoffActive(false)
    , _paused(false)
{}

// =============================================================================
// begin() — call from setup()
// =============================================================================

void MotionController::begin() {
#ifdef ARDUINO
    // setup() initialises the board — PID tuning, current, direction.
    // Acceleration and velocity are NOT set here; they are set per-move
    // via setMaxVelocity() / setMaxAcceleration() in startMoveToMm().
    //
    // Parameters:
    //   mode              — NORMAL (closed-loop position control)
    //   stepsPerRevolution— must match MOTOR_STEPS_PER_REV in config.h
    //   pTerm/iTerm/dTerm — PID defaults; tune on hardware (start 10/0/0)
    //   dropinStepSize    — unused in NORMAL mode
    //   setHome           — false: we set home explicitly after homing move
    //   invert            — VERIFY: set 1 if motor direction is reversed
    //   runCurrent        — % of rated current while moving
    //   holdCurrent       — % of rated current while stationary
    _stepper.setup(
        NORMAL,                 // mode
        MOTOR_STEPS_PER_REV,    // 200 — must match config.h
        10.0f,                  // pTerm  (tune on hardware)
        0.0f,                   // iTerm
        0.0f,                   // dTerm
        16,                     // dropinStepSize (irrelevant in NORMAL mode)
        false,                  // setHome — we call encoder.setHome() after homing
        0,                      // invert  — VERIFY wiring: set 1 to flip direction
        50,                     // runCurrent  [% of rated current]
        30                      // holdCurrent [% of rated current]
    );
#endif
}

// =============================================================================
// update() — call every loop()
// =============================================================================

void MotionController::update() {
    switch (_mode) {
        case ProfileMode::HOMING:      updateHoming();      break;
        case ProfileMode::TRAPEZOIDAL: updateTrapezoidal(); break;
        case ProfileMode::SEGMENTED:   updateSegmented();   break;
        case ProfileMode::JOG:         /* library handles it */   break;
        case ProfileMode::NONE:        /* idle */           break;
    }
}

// =============================================================================
// Commands
// =============================================================================

void MotionController::executeHome() {
    _mode = ProfileMode::HOMING;
    _homingBackoffActive = false;
    _commandedVelocityMms = HOMING_SPEED_MM_S;

#ifdef ARDUINO
    // CCW drives the carriage UP toward the top endstop (home position).
    // VERIFY: swap to CW if wiring makes CCW go down instead.
    _stepper.setMaxAcceleration(HOMING_SPEED_MM_S * STEPS_PER_MM);
    _stepper.setMaxDeceleration(HOMING_SPEED_MM_S * STEPS_PER_MM);
    _stepper.setMaxVelocity(HOMING_SPEED_MM_S * STEPS_PER_MM);
    _stepper.runContinous(CCW);  // NOTE: library spells it "Continous" (one 'u')
#endif
}

void MotionController::setSoftLimits(float minMm, float maxMm) {
    _softLimitMinMm = minMm;
    _softLimitMaxMm = maxMm;
}

void MotionController::setDebug(bool on) {
    _debugOn = on;
}

void MotionController::stop() {
    _mode = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell = false;
    _sm.toReady();

#ifdef ARDUINO
    // VERIFY_API: SOFT stop — decelerates using the current accel ramp.
    _stepper.stop(SOFT);
#endif
}

void MotionController::estop() {
    _mode = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell = false;
    _paused = false;

    // ESTOP can be called from any state — drive state to ERROR.
    _sm.toError(ErrorCode::NONE);

#ifdef ARDUINO
    // VERIFY_API: HARD stop — immediate, no deceleration ramp.
    _stepper.stop(HARD);
#endif
}

void MotionController::pause() {
    // Save enough state to resume from the same point.
    _pauseSnapshot.mode       = _mode;
    _pauseSnapshot.phase      = _sm.getPhase();
    _pauseSnapshot.currentDip = _currentDip;
    _pauseSnapshot.segIndex   = _segIndex;
    _pauseSnapshot.segDip     = _segCurrentDip;

    _paused = true;
    _mode   = ProfileMode::NONE;
    _inDwell = false;

    _sm.toPaused();

#ifdef ARDUINO
    _stepper.stop(SOFT);
#endif
}

void MotionController::resume() {
    if (!_paused) return;

    _paused = false;
    _mode   = _pauseSnapshot.mode;

    _sm.toRunning();
    _sm.setPhase(_pauseSnapshot.phase);

    if (_mode == ProfileMode::TRAPEZOIDAL) {
        _currentDip = _pauseSnapshot.currentDip;
        // Re-issue the move for the current phase from current position.
        // The library will ramp up from rest.
        RunPhase ph = _pauseSnapshot.phase;
        if (ph == RunPhase::DESCENDING) {
            startMoveToMm(_depthMm, _dipSpeedMms);
        } else if (ph == RunPhase::ASCENDING) {
            startMoveToMm(0.0f, _withdrawSpeedMms);
        } else if (ph == RunPhase::DWELL_BOTTOM) {
            startDwell(_dwellBottomMs);
        } else if (ph == RunPhase::DWELL_TOP) {
            startDwell(_dwellTopMs);
        }
    } else if (_mode == ProfileMode::SEGMENTED) {
        _segIndex      = _pauseSnapshot.segIndex;
        _segCurrentDip = _pauseSnapshot.segDip;
        RunPhase ph = _pauseSnapshot.phase;
        if (ph == RunPhase::DESCENDING || ph == RunPhase::ASCENDING) {
            // Restart current segment from current position
            if (_segIndex < _segCount) {
                float target = getPositionMm() + _segments[_segIndex].distMm;
                startMoveToMm(target, _segments[_segIndex].speedMms);
            }
        } else if (ph == RunPhase::DWELL_BOTTOM) {
            startDwell(_segDwellBottomMs);
        } else if (ph == RunPhase::DWELL_TOP) {
            startDwell(_segDwellTopMs);
        }
    }
}

void MotionController::jog(bool up, float speedMms) {
    _mode = ProfileMode::JOG;
    _commandedVelocityMms = speedMms;

#ifdef ARDUINO
    _stepper.setMaxAcceleration(_accelMms2 * STEPS_PER_MM);
    _stepper.setMaxDeceleration(_accelMms2 * STEPS_PER_MM);
    _stepper.setMaxVelocity(speedMms * STEPS_PER_MM);
    // CCW = up, CW = down — matches homing direction.
    // VERIFY: swap CW/CCW if wiring is inverted.
    _stepper.runContinous(up ? CCW : CW);
#endif
}

void MotionController::runProfile(float dipSpeedMms, float withdrawSpeedMms,
                                   float accelMms2, float depthMm,
                                   int dwellBottomMs, int dwellTopMs, int nDips) {
    _dipSpeedMms     = dipSpeedMms;
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
    // Mode stays NONE until runLoadedMove() is called.
}

bool MotionController::addSegment(float distMm, float speedMms) {
    if (_segCount >= MOVE_SEG_BUFFER_SIZE) return false;
    _segments[_segCount].distMm  = distMm;
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

    // Start first segment
    float target = getPositionMm() + _segments[0].distMm;
    startMoveToMm(target, _segments[0].speedMms);
}

// =============================================================================
// ISR callback
// =============================================================================

void MotionController::onEndstopTriggered(bool isTop) {
    if (_mode == ProfileMode::HOMING && isTop && !_homingBackoffActive) {
        // Expected: top endstop during homing (homing moves UP).
        // Stop, zero encoder at this position (home = 0), back off downward.
        _commandedVelocityMms = 0.0f;

#ifdef ARDUINO
        _stepper.stop(HARD);
        // Zero the encoder reference. From here: positive angle = down (dip direction).
        _stepper.encoder.setHome();
#endif

        // Back off downward by HOMING_BACKOFF_MM (positive mm = away from home).
        _homingBackoffActive = true;
        startMoveToMm(HOMING_BACKOFF_MM, HOMING_SPEED_MM_S);
        return;
    }

    // Any other endstop trigger is unexpected — E-stop and raise error.
    estop();
    _sm.toError(ErrorCode::ENDSTOP_TRIGGERED_UNEXPECTEDLY);
}

// =============================================================================
// Telemetry interface
// =============================================================================

float MotionController::getPositionMm() {
#ifdef ARDUINO
    return positionMm();
#else
    return 0.0f;
#endif
}

float MotionController::getActualVelocityMms() {
#ifdef ARDUINO
    return actualVelocityMms();
#else
    return 0.0f;
#endif
}

float MotionController::getCommandedVelocityMms() const {
    return _commandedVelocityMms;
}

float MotionController::getActualAccelMms2() const {
    // The library doesn't expose instantaneous acceleration directly.
    // Return 0 — telemetry can differentiate velocity over time if needed.
    return 0.0f;
}

// =============================================================================
// Private helpers
// =============================================================================

void MotionController::startMoveToMm(float targetMm, float speedMms) {
    _targetMm             = targetMm;
    _commandedVelocityMms = speedMms;

#ifdef ARDUINO
    // setMaxAcceleration/Deceleration: steps/s²   setMaxVelocity: steps/s
    _stepper.setMaxAcceleration(_accelMms2 * STEPS_PER_MM);
    _stepper.setMaxDeceleration(_accelMms2 * STEPS_PER_MM);
    _stepper.setMaxVelocity(speedMms * STEPS_PER_MM);
    _stepper.moveToAngle(mmToDeg(targetMm));
#endif
}

void MotionController::startDwell(uint32_t ms) {
    _inDwell        = true;
    _dwellDurationMs = ms;
#ifdef ARDUINO
    _dwellStartMs   = millis();
#else
    _dwellStartMs   = 0;
#endif
    _commandedVelocityMms = 0.0f;
}

bool MotionController::isDwellComplete() const {
    if (!_inDwell) return false;
#ifdef ARDUINO
    return (millis() - _dwellStartMs) >= _dwellDurationMs;
#else
    return true;
#endif
}

bool MotionController::isMoveComplete() {
    if (_inDwell) return false;
#ifdef ARDUINO
    return _stepper.getMotorState(STANDSTILL);
#else
    return false;
#endif
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
// updateHoming()
// =============================================================================

void MotionController::updateHoming() {
    if (!_homingBackoffActive) {
        // Still driving up — waiting for onEndstopTriggered() ISR.
        return;
    }

    // Backoff move in progress — wait for it to complete.
    if (isMoveComplete()) {
        _mode = ProfileMode::NONE;
        _commandedVelocityMms = 0.0f;
        _sm.toReady();
    }
}

// =============================================================================
// updateTrapezoidal()
// =============================================================================

void MotionController::updateTrapezoidal() {
    RunPhase phase = _sm.getPhase();

    if (_inDwell) {
        if (!isDwellComplete()) return;
        _inDwell = false;

        if (phase == RunPhase::DWELL_BOTTOM) {
            // Dwell at bottom complete — start ascending
            _sm.setPhase(RunPhase::ASCENDING);
            startMoveToMm(0.0f, _withdrawSpeedMms);

        } else if (phase == RunPhase::DWELL_TOP) {
            // Dwell at top complete — start next dip
            _sm.setPhase(RunPhase::DESCENDING);
            startMoveToMm(_depthMm, _dipSpeedMms);
        }
        return;
    }

    if (!isMoveComplete()) return;

    if (phase == RunPhase::DESCENDING) {
        // Arrived at depth — dwell at bottom
        _sm.setPhase(RunPhase::DWELL_BOTTOM);
        startDwell(_dwellBottomMs);

    } else if (phase == RunPhase::ASCENDING) {
        // Arrived at top
        if (_currentDip < _nDips) {
            // More dips to go — dwell at top then repeat
            _currentDip++;
            _sm.setPhase(RunPhase::DWELL_TOP);
            startDwell(_dwellTopMs);
        } else {
            // All dips complete
            _mode = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.setPhase(RunPhase::NONE);
            _sm.toReady();
        }
    }
}

// =============================================================================
// updateSegmented()
// =============================================================================

void MotionController::updateSegmented() {
    RunPhase phase = _sm.getPhase();

    if (_inDwell) {
        if (!isDwellComplete()) return;
        _inDwell = false;

        if (phase == RunPhase::DWELL_BOTTOM) {
            // Finished bottom dwell — start ascending (segments from current index)
            _sm.setPhase(RunPhase::ASCENDING);
            if (_segIndex < _segCount) {
                float target = getPositionMm() + _segments[_segIndex].distMm;
                startMoveToMm(target, _segments[_segIndex].speedMms);
            }

        } else if (phase == RunPhase::DWELL_TOP) {
            // Finished top dwell — start next dip from segment 0
            _segIndex = 0;
            _sm.setPhase(RunPhase::DESCENDING);
            float target = getPositionMm() + _segments[0].distMm;
            startMoveToMm(target, _segments[0].speedMms);
        }
        return;
    }

    if (!isMoveComplete()) return;

    // Current segment complete — advance
    _segIndex++;

    if (_segIndex < _segCount) {
        // More segments: continue executing
        float target = getPositionMm() + _segments[_segIndex].distMm;
        startMoveToMm(target, _segments[_segIndex].speedMms);
        return;
    }

    // All segments in this half exhausted — transition to dwell
    if (phase == RunPhase::DESCENDING) {
        _sm.setPhase(RunPhase::DWELL_BOTTOM);
        startDwell(_segDwellBottomMs);
        // Reset index to start of segment list for the ascent half
        _segIndex = 0;

    } else if (phase == RunPhase::ASCENDING) {
        // One full dip complete
        if (_segCurrentDip < _segNDips) {
            _segCurrentDip++;
            _sm.setPhase(RunPhase::DWELL_TOP);
            startDwell(_segDwellTopMs);
        } else {
            // All dips done
            _mode = ProfileMode::NONE;
            _commandedVelocityMms = 0.0f;
            _sm.setPhase(RunPhase::NONE);
            _sm.toReady();
        }
    }
}
