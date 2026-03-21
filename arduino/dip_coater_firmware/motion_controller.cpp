#include "motion_controller.h"
#include "trace.h"

// =============================================================================
// Unit conversion helpers
// =============================================================================

/** @brief Convert linear position (mm) to motor angle (degrees). */
float MotionController::mmToDeg(float mm) const {
    // The leadscrew converts one full revolution (360°) into LEADSCREW_MM_PER_REV
    // of linear travel.  Dividing 360 by that pitch gives degrees per mm.
    return mm * (360.0f / LEADSCREW_MM_PER_REV);
}

/** @brief Return current encoder position in mm relative to home. */
float MotionController::positionMm() {
    // angleMoved() returns cumulative shaft rotation in degrees since setHome().
    // Multiplying by (mm_per_rev / 360) converts to linear mm.
    return _stepper.angleMoved() * (LEADSCREW_MM_PER_REV / 360.0f);
}

/** @brief Return actual motor velocity in mm/s from the encoder RPM. */
float MotionController::actualVelocityMms() {
    // getRPM() gives shaft revolutions per minute.
    // Multiplying by (mm_per_rev / 60) converts to mm per second.
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
    , _jogUp              (false)
    , _targetMm           (0.0f)
    , _moveStartMm        (0.0f)
    , _profileStartMm     (0.0f)
    , _movingSpeedMms     (0.0f)
    , _movingAccelMms2    (0.0f)
    , _moveStartMs        (0)
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
    // Initialise the stepper driver with open-loop positioning mode (NORMAL).
    // Parameters: mode, steps/rev, P/I/D gains, microstep size,
    //             setHome, invertDirection, runCurrent%, holdCurrent%.
    // invert=0: motor moves up on CW command — swap to 1 if wiring is reversed.
    _stepper.setup(NORMAL, MOTOR_STEPS_PER_REV,
                   10.0f, 0.0f, 0.0f,
                   16, false, 0, 50, 30);

    // TPWMTHRS sets the velocity crossover between StealthChop (quiet) and
    // SpreadCycle (higher torque).  Below this threshold the driver uses
    // StealthChop; above it switches to SpreadCycle automatically.
    // STEALTH_TPWMTHRS is tuned in config.h for ~3 mm/s crossover.
    _stepper.driver.writeRegister(TPWMTHRS,   STEALTH_TPWMTHRS);

    // TPOWERDOWN delays the reduction to hold current after the motor stops.
    // A short delay prevents the audible click caused by an immediate current
    // drop while the rotor is still settling against the load.
    _stepper.driver.writeRegister(TPOWERDOWN, STEALTH_TPOWERDOWN);
}

// =============================================================================
// update()  — call every loop()
// =============================================================================

void MotionController::update() {
    // Dispatch to the handler for whichever motion mode is currently active.
    // JOG and NONE have no update logic — the library drives continuous motion
    // for JOG, and NONE means the motor is idle.
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
    // so StealthChop stays active — runContinuous forces SpreadCycle regardless
    // of TPWMTHRS.  Command more than the maximum travel distance upward; the
    // top-endstop ISR will hard-stop and zero the encoder when it fires.
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
    // Reset internal mode and dwell state so update() does nothing,
    // then issue a soft-stop to let the motor decelerate naturally.
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _sm.toReady();
    _stepper.stop(SOFT);
}

void MotionController::estop() {
    // Hard-stop cuts power immediately — no deceleration ramp.
    // Also clears the paused flag so a stale resume() can't restart motion,
    // and clears any in-progress limit-backoff so it doesn't restart next loop.
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _paused               = false;
    _limitTriggered       = false;
    _limitBackoffActive   = false;
    _sm.toError(ErrorCode::NONE);
    _stepper.stop(HARD);
}

void MotionController::pause() {
    // Snapshot the entire motion state so resume() can restart exactly
    // where we left off, regardless of which profile mode was active.
    _pauseSnapshot.mode       = _mode;
    _pauseSnapshot.phase      = _sm.getPhase();
    _pauseSnapshot.currentDip = _currentDip;
    _pauseSnapshot.segIndex   = _segIndex;

    // Clear the active mode so update() stops running the profile,
    // then soft-stop the motor and enter PAUSED state.
    // Also clear limit-backoff flags so the backoff is not re-entered on resume.
    _paused             = true;
    _mode               = ProfileMode::NONE;
    _inDwell            = false;
    _limitTriggered     = false;
    _limitBackoffActive = false;
    _sm.toPaused();
    _stepper.stop(SOFT);
}

void MotionController::resume() {
    // Guard: only proceed if a valid pause snapshot exists.
    if (!_paused) return;

    // Restore mode and state machine before re-issuing the motion command.
    _paused = false;
    _mode   = _pauseSnapshot.mode;
    _sm.toRunning();
    _sm.setPhase(_pauseSnapshot.phase);

    if (_mode == ProfileMode::TRAPEZOIDAL) {
        // Restore the dip counter, then restart whichever phase was active.
        // For dwell phases, restart the dwell timer from zero (the remaining
        // dwell time is not tracked — the full dwell repeats on resume).
        _currentDip = _pauseSnapshot.currentDip;
        RunPhase ph = _pauseSnapshot.phase;
        if      (ph == RunPhase::DESCENDING)   startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
        else if (ph == RunPhase::ASCENDING)    startMoveToMm(_profileStartMm,            _withdrawSpeedMms);
        else if (ph == RunPhase::DWELL_BOTTOM) startDwell(_dwellBottomMs);
        else if (ph == RunPhase::DWELL_TOP)    startDwell(_dwellTopMs);

    } else if (_mode == ProfileMode::SEGMENTED) {
        // Restore the segment index, then restart the current segment.
        // For MOVE segments the motor re-commands from the current position.
        // For DWELL segments the dwell timer restarts from zero.
        _segIndex = _pauseSnapshot.segIndex;
        if (_segIndex < _segCount) {
            startCurrentSegment();
        }

    } else if (_mode == ProfileMode::MOVING) {
        // Resume a paused CMD MOVE: re-command to the original absolute target.
        // _targetMm was set by the original moveByMm() call and is still valid.
        // Restore the move's own accel before commanding the move.
        _accelMms2 = _movingAccelMms2;
        startMoveToMm(_targetMm, _movingSpeedMms);

    } else if (_mode == ProfileMode::JOG) {
        // Resume a paused jog: restart continuous motion in the same direction
        // at the same speed.  _commandedVelocityMms and _jogUp were saved in jog().
        setSpeed(_commandedVelocityMms);
        _stepper.runContinous(_jogUp ? CW : CCW);

    } else if (_mode == ProfileMode::LIMIT_BACKOFF) {
        // A limit-backoff was interrupted by pause().  The backoff is a safety
        // move — do not re-attempt it.  Instead, clear the mode and return to
        // READY so the user can issue a fresh command.
        _mode = ProfileMode::NONE;
        _commandedVelocityMms = 0.0f;
        _sm.toReady();   // overrides the toRunning() called above
    }
}

void MotionController::jog(bool up, float speedMms) {
    // Save direction so resume() can restart the jog if it was paused.
    _jogUp                = up;
    _mode                 = ProfileMode::JOG;
    _commandedVelocityMms = speedMms;
    _accelMms2            = DEFAULT_ACCEL_MM_S2;
    setSpeed(speedMms);
    // runContinous drives the motor indefinitely in the given direction.
    // CW = upward (toward home), CCW = downward — matches the coordinate system.
    _stepper.runContinous(up ? CW : CCW);
}

void MotionController::moveByMm(float deltaMm, float speedMms, float accelMms2) {
    // Save the move parameters so resume() can restart if the move is paused.
    _movingSpeedMms  = speedMms;
    _movingAccelMms2 = accelMms2;

    // Temporarily apply this move's accel for the startMoveToMm() call, then
    // restore the previous value so the trapezoidal/segmented profile params
    // are not clobbered if moveByMm() is called from Diagnostics mid-profile.
    float saved  = _accelMms2;
    _accelMms2   = accelMms2;
    _mode        = ProfileMode::MOVING;

    // Convert the relative displacement to an absolute target by adding the
    // current encoder position.  startMoveToMm() checks the soft limit.
    startMoveToMm(positionMm() + deltaMm, speedMms);
    _accelMms2   = saved;
}

void MotionController::runProfile(float    dipSpeedMms,   float withdrawSpeedMms,
                                   float    accelMms2,     float depthMm,
                                   uint32_t dwellBottomMs, uint32_t dwellTopMs,
                                   int      nDips) {
    // Store all profile parameters for use throughout the multi-dip sequence.
    _dipSpeedMms      = dipSpeedMms;
    _withdrawSpeedMms = withdrawSpeedMms;
    _accelMms2        = accelMms2;
    _depthMm          = depthMm;
    _dwellBottomMs    = dwellBottomMs;
    _dwellTopMs       = dwellTopMs;
    _nDips            = nDips;
    _currentDip       = 1;

    // Record the position at profile start so all dips use the same top
    // reference point regardless of small encoder drift.
    _profileStartMm   = positionMm();

    // Transition to RUNNING and immediately start the first descent.
    // The dip target is _profileStartMm minus depthMm (downward = negative).
    _mode             = ProfileMode::TRAPEZOIDAL;
    _sm.toRunning();
    _sm.setPhase(RunPhase::DESCENDING);
    TR2F("runProfile depth=", _depthMm, " dipSpd=", _dipSpeedMms);
    startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
}

void MotionController::beginSegmentedMove() {
    // Clear the segment buffer so addSegment() starts filling from the beginning.
    // The move does not start until runLoadedMove() is called.
    _segCount = 0;
    _segIndex = 0;
}

bool MotionController::addSegment(float distMm, float speedMms, float accelMms2) {
    if (_segCount >= MOVE_SEG_BUFFER_SIZE) return false;
    _segments[_segCount].type     = Segment::Type::MOVE;
    _segments[_segCount].distMm   = distMm;
    _segments[_segCount].speedMms = speedMms;
    _segments[_segCount].accelMms2 = accelMms2;
    _segments[_segCount].dwellMs  = 0;
    _segCount++;
    return true;
}

bool MotionController::addDwellSegment(uint32_t dwellMs) {
    if (_segCount >= MOVE_SEG_BUFFER_SIZE) return false;
    _segments[_segCount].type    = Segment::Type::DWELL;
    _segments[_segCount].dwellMs = dwellMs;
    // Motion fields unused for DWELL segments
    _segments[_segCount].distMm   = 0.0f;
    _segments[_segCount].speedMms = 0.0f;
    _segments[_segCount].accelMms2 = 0.0f;
    _segCount++;
    return true;
}

void MotionController::runLoadedMove() {
    if (_segCount == 0) return;   // nothing to run

    _segIndex = 0;
    _mode     = ProfileMode::SEGMENTED;
    _sm.toRunning();
    _sm.setPhase(RunPhase::NONE);
    startCurrentSegment();   // dispatches to move or dwell based on segment type
}

// =============================================================================
// ISR callback
// =============================================================================

void MotionController::onEndstopTriggered(bool isTop) {
    // ---- Expected trigger: top endstop during homing -------------------------
    // Zero the encoder at this position and start the backoff move.
    // The backoff is a short downward move to release the endstop mechanism.
    if (_mode == ProfileMode::HOMING && isTop && !_homingBackoffActive) {
        _commandedVelocityMms = 0.0f;
        _stepper.stop(HARD);
        _stepper.encoder.setHome();   // set encoder origin at the top endstop
        _homingBackoffActive = true;
        startMoveToMm(-HOMING_BACKOFF_MM, HOMING_SPEED_MM_S);
        return;
    }

    // ---- Ignore benign triggers ----------------------------------------------
    // NONE = motor idle, LIMIT_BACKOFF = already handling a prior trigger.
    if (_mode == ProfileMode::NONE || _mode == ProfileMode::LIMIT_BACKOFF) return;

    // ---- Unexpected trigger during motion ------------------------------------
    // Hard-stop immediately to prevent mechanical damage, then schedule a
    // short backoff move away from the endstop.  The backoff is executed in
    // updateLimitBackoff() on the next loop() iteration (not here in the ISR)
    // because startMoveToMm() is not ISR-safe on all platforms.
    // The encoder origin (set by CMD HOME) is intentionally NOT touched here —
    // the absolute backoff target in updateLimitBackoff() references it.
    _stepper.stop(HARD);
    _commandedVelocityMms = 0.0f;
    _inDwell              = false;
    _paused               = false;
    _limitTriggered       = true;     // flag for updateLimitBackoff() to process
    _limitIsTop           = isTop;
    _mode                 = ProfileMode::LIMIT_BACKOFF;
}

// =============================================================================
// Telemetry accessors
// =============================================================================

float MotionController::getPositionMm()                { return positionMm();          }
float MotionController::getActualVelocityMms()         { return actualVelocityMms();   }
float MotionController::getCommandedVelocityMms() const{ return _commandedVelocityMms; }
float MotionController::getActualAccelMms2()      const{ return 0.0f;                  }  // not yet implemented

bool MotionController::isMoving() {
    // getMotorState(STANDSTILL) returns 1 while the motor is actively stepping
    // and 0 when it has stopped.  See isMoveComplete() for the full explanation.
    return _stepper.getMotorState(STANDSTILL);
}

// =============================================================================
// Private helpers
// =============================================================================

void MotionController::startMoveToMm(float targetMm, float speedMms) {
    // Reject the move if the target is outside the soft limits.
    // checkSoftLimit() calls estop() and transitions to ERROR internally.
    if (!checkSoftLimit(targetMm)) return;

    // Record start position and time for the isMoveComplete() guards.
    _moveStartMm          = positionMm();
    _moveStartMs          = millis();
    _targetMm             = targetMm;
    _commandedVelocityMms = speedMms;
    TR2F("startMoveToMm pos=", _moveStartMm, " target=", targetMm);

    // Apply speed/accel, then command an absolute angle move.
    // moveToAngle() uses the encoder origin set by setHome() during homing,
    // so _targetMm (converted to degrees) is an absolute shaft position.
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
    // Never complete during a dwell — dwell and move are mutually exclusive.
    if (_inDwell) return false;

    // STANDSTILL semantics (confirmed empirically):
    //   getMotorState(STANDSTILL) == 1  →  motor is actively stepping
    //   getMotorState(STANDSTILL) == 0  →  motor has stopped
    // This is inverted from the register name, but matches observed behaviour.
    // Return false (not complete) while the motor is still stepping.
    if (_stepper.getMotorState(STANDSTILL)) return false;

    float pos      = positionMm();
    float travelMm = fabsf(_targetMm  - _moveStartMm);
    float errorMm  = fabsf(pos        - _targetMm);

    // Guard 1: Reject a spurious early STANDSTILL within the first 100 ms of
    // motion.  Brief glitches in the STANDSTILL flag can occur at the start of
    // any move (including short ones) before the ramp has fully built up.
    // Using a time guard instead of a distance fraction covers all move lengths.
    if ((millis() - _moveStartMs) < 100UL) return false;

    // Guard 2: Reject if the motor stopped far from the target.  This catches
    // cases where an unexpected endstop trigger halted the motor early —
    // without this guard the profile would advance to the next phase prematurely.
    // Only applied for moves longer than 0.5 mm to avoid false failures on
    // deliberate short nudges where a 3 mm error threshold is meaningless.
    if (travelMm > 0.5f && errorMm > 3.0f) return false;

    TR2F("isMoveComplete pos=", pos, " target=", _targetMm);
    return true;
}

bool MotionController::checkSoftLimit(float targetMm) {
    if (targetMm < _softLimitMinMm || targetMm > _softLimitMaxMm) {
        // Print a diagnostic message with the offending target and current limits.
        Serial.print("ERR SOFT_LIMIT_EXCEEDED target=");
        Serial.print(targetMm, 2);
        Serial.print("mm limits=[");
        Serial.print(_softLimitMinMm, 2);
        Serial.print(", ");
        Serial.print(_softLimitMaxMm, 2);
        Serial.println("]");
        // estop() hard-stops the motor; toError() overrides the ErrorCode with
        // the specific fault so the caller can identify the cause.
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
    // Waiting for the top endstop ISR to fire and set _homingBackoffActive.
    if (!_homingBackoffActive) return;

    // Backoff move in progress — wait for it to finish.
    if (!isMoveComplete()) return;

    // Backoff complete — homing done.
    TR("homing: backoff done -> READY");
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _sm.toReady();
}

// =============================================================================
// updateTrapezoidal
// =============================================================================

void MotionController::updateTrapezoidal() {
    RunPhase phase = _sm.getPhase();

    // ---- Dwell in progress --------------------------------------------------
    // Remain here until the dwell timer expires, then advance to the next phase.
    if (_inDwell) {
        if (!isDwellComplete()) return;
        _inDwell = false;

        if (phase == RunPhase::DWELL_BOTTOM) {
            // Bottom dwell complete → start ascending back to profile start.
            TR("dwell_bottom done -> ASCENDING");
            _sm.setPhase(RunPhase::ASCENDING);
            startMoveToMm(_profileStartMm, _withdrawSpeedMms);
        } else if (phase == RunPhase::DWELL_TOP) {
            // Top dwell complete → start the next descent.
            TR("dwell_top done -> DESCENDING");
            _sm.setPhase(RunPhase::DESCENDING);
            startMoveToMm(_profileStartMm - _depthMm, _dipSpeedMms);
        }
        return;
    }

    // ---- Move in progress ---------------------------------------------------
    // Remain here until the motor reaches the target position.
    if (!isMoveComplete()) return;

    // ---- Move complete — advance the phase ----------------------------------
    if (phase == RunPhase::DESCENDING) {
        // Reached the bottom — begin dwell in solution.
        TR("DESCENDING done -> DWELL_BOTTOM");
        _sm.setPhase(RunPhase::DWELL_BOTTOM);
        startDwell(_dwellBottomMs);

    } else if (phase == RunPhase::ASCENDING) {
        if (_currentDip < _nDips) {
            // More dips remain — dwell at the top before the next descent.
            _currentDip++;
            TR("ASCENDING done -> DWELL_TOP");
            _sm.setPhase(RunPhase::DWELL_TOP);
            startDwell(_dwellTopMs);
        } else {
            // All dips complete — return to READY.
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
//
// Executes a pre-loaded sequence of segments (distance + speed + accel) one
// after another.  There is no dip-cycle structure — segments are simply run
// in order and the move ends when the last segment completes.
//
// =============================================================================

// =============================================================================
// startCurrentSegment  — private helper
// =============================================================================

void MotionController::startCurrentSegment() {
    const Segment& seg = _segments[_segIndex];
    if (seg.type == Segment::Type::DWELL) {
        // Hold-position segment — no motion, just start the dwell timer.
        startDwell(seg.dwellMs);
    } else {
        // Motion segment — ensure the dwell flag is clear before starting the
        // move, so isMoveComplete() is not gated by a stale _inDwell value.
        _inDwell     = false;
        _accelMms2   = seg.accelMms2;
        float target = getPositionMm() + seg.distMm;
        startMoveToMm(target, seg.speedMms);
    }
}

// =============================================================================
// updateSegmented
// =============================================================================
//
// Executes a pre-loaded sequence of MOVE and DWELL segments in order.
// Each segment completes independently (move reaches target, or dwell timer
// expires) before the next one begins.
//
// =============================================================================

void MotionController::updateSegmented() {
    const Segment& seg = _segments[_segIndex];

    // ---- Check for completion of the current segment ------------------------
    if (seg.type == Segment::Type::DWELL) {
        // Waiting for the dwell timer set by startCurrentSegment().
        if (!isDwellComplete()) return;
        _inDwell = false;
    } else {
        // Waiting for the motor to reach the move target.
        if (!isMoveComplete()) return;
    }

    // ---- Current segment complete — advance to the next one -----------------
    _segIndex++;
    if (_segIndex < _segCount) {
        startCurrentSegment();
        return;
    }

    // ---- All segments complete — return to READY ----------------------------
    _mode                 = ProfileMode::NONE;
    _commandedVelocityMms = 0.0f;
    _sm.setPhase(RunPhase::NONE);
    _sm.toReady();
    Serial.println("CMD:SEGMENTED:DONE");
}

// =============================================================================
// updateLimitBackoff
// =============================================================================

void MotionController::updateLimitBackoff() {
    uint32_t now = millis();

    // ---- First pass: process the pending trigger flag -----------------------
    // The ISR sets _limitTriggered and switches _mode to LIMIT_BACKOFF, but
    // does not call startMoveToMm() because that function is not ISR-safe.
    // On the first loop() call after the ISR, we pick up the flag here.
    if (_limitTriggered) {
        _limitTriggered = false;
        Serial.print(_limitIsTop ? "TOP" : "BOTTOM");
        Serial.println(" LIMIT switch triggered");

        // Back off away from whichever endstop fired.
        // Coordinate convention: home (top) = 0, positive = up, negative = down.
        //
        //   Top triggered    → use an ABSOLUTE target of -LIMIT_BACKOFF_MM.
        //                      The top endstop IS the home position (0 mm), so
        //                      this is always exactly LIMIT_BACKOFF_MM below the
        //                      physical switch regardless of how far the motor
        //                      overshot before stopping.  positionMm() is NOT
        //                      used because at 50 mm/s the motor may have coasted
        //                      +7 mm past the switch, making a relative backoff
        //                      land above the switch rather than below it.
        //                      The encoder origin set by CMD HOME is preserved.
        //
        //   Bottom triggered → no calibrated reference; use a relative backoff
        //                      from wherever the motor stopped.
        float target = _limitIsTop
            ? -LIMIT_BACKOFF_MM
            : positionMm() + LIMIT_BACKOFF_MM;
        _limitBackoffActive = true;
        _limitBackoffStart  = now;
        startMoveToMm(target, LIMIT_BACKOFF_SPEED_MM_S);
        return;
    }

    // ---- Subsequent passes: wait for backoff to complete -------------------
    if (_limitBackoffActive) {
        uint32_t elapsed = now - _limitBackoffStart;

        // Ignore isMoveComplete() for the first 200 ms — the STANDSTILL signal
        // may briefly read "stopped" before the motor has started moving.
        if (elapsed < 200) return;

        // Once the backoff is done (or a 5 s safety timeout expires), return
        // to READY so the user can issue a new command.
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
    // Skip if the state machine is not RUNNING.  This prevents Diagnostics
    // from accidentally completing a CMD MOVE that was never started — the
    // Diagnostics module monitors its own completion via isMoving().
    if (_sm.getState() != SystemState::RUNNING) return;

    // When the motor reaches the target, release the RUNNING state and print
    // a confirmation so the Python UI knows the move is done.
    if (isMoveComplete()) {
        _mode                 = ProfileMode::NONE;
        _commandedVelocityMms = 0.0f;
        _sm.toReady();
        Serial.println("CMD:MOVE:DONE");
    }
}
