#include "diagnostics.h"
#include "trace.h"

// =============================================================================
// Constructor / begin
// =============================================================================

Diagnostics::Diagnostics()
    : _mc(nullptr)
    , _sm(nullptr)
    , _mode(Mode::INACTIVE)
    , _motorPhase(MotorPhase::JOG_DOWN)
    , _phaseStart(0)
    , _startPos(0.0f)
    , _passed(0)
    , _failed(0)
    , _checkEncoder(false)
    , _lastTop(false)
    , _lastBot(false)
    , _moveTestStartPos(0.0f)
    , _moveTestDeltaMm(0.0f)
    , _moveTestSpeedMms(DIAG_SPEED_MMS)
    , _moveTestAccelMms2(DIAG_ACCEL_MMS2)
    , _moveTestStart(0)
    , _moveStarted(false)
    , _trackedPos(0.0f)
    , _stableSince(0)
    , _calStartPos(0.0f)
    , _calCommandedMm(0.0f)
    , _calEncoderMm(0.0f)
    , _calStart(0)
{}

void Diagnostics::begin(MotionController& mc, StateMachine& sm) {
    // Store pointers to the shared subsystems.  These are passed in (not
    // constructed here) so the Diagnostics object can be declared globally
    // in the .ino without a complex constructor.
    _mc = &mc;
    _sm = &sm;
}

// =============================================================================
// Public interface
// =============================================================================

void Diagnostics::startMotorTest() {
    // Print the test header so the user can see what parameters are in use.
    Serial.print("DIAG:MOTOR:START down=");
    Serial.print(DIAG_DIST_MM, 0);
    Serial.print("mm speed=");
    Serial.print(DIAG_SPEED_MMS, 0);
    Serial.println("mm/s");

    // Initialise phase state: MOVE_DOWN phase, reset pass/fail counters,
    // record the starting position, and kick off the first move.
    _mode         = Mode::MOTOR_TEST;
    _motorPhase   = MotorPhase::MOVE_DOWN;
    _passed       = 0;
    _failed       = 0;
    _checkEncoder = false;   // encoder accuracy check disabled for plain MOTOR test
    _startPos     = _mc->getPositionMm();
    _phaseStart   = millis();
    _moveStarted  = false;

    _mc->moveByMm(DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
}

void Diagnostics::startMotorEncoderTest() {
    // Same sequence as startMotorTest() but with encoder checking enabled.
    // Each move's actual displacement is compared to the commanded distance
    // within TOLERANCE_MM to generate PASS/FAIL results.
    Serial.print("DIAG:MOTORENCODER:START down=");
    Serial.print(DIAG_DIST_MM, 0);
    Serial.print("mm speed=");
    Serial.print(DIAG_SPEED_MMS, 0);
    Serial.println("mm/s");

    _mode         = Mode::MOTOR_TEST;
    _motorPhase   = MotorPhase::MOVE_DOWN;
    _passed       = 0;
    _failed       = 0;
    _checkEncoder = true;    // encoder accuracy check enabled
    _startPos     = _mc->getPositionMm();
    _phaseStart   = millis();
    _moveStarted  = false;

    _mc->moveByMm(DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
}

void Diagnostics::startEndstopTest() {
    // Print instructions, then seed _lastTop/_lastBot with the current
    // electrical state so the first change is detected as a real transition.
    Serial.println("DIAG:ENDSTOP:START");
    Serial.println("DIAG:ENDSTOP:Trigger each endstop manually. Send DIAG EXIT to stop.");

    _mode    = Mode::ENDSTOP_TEST;
    _lastTop = digitalRead(PIN_ENDSTOP_TOP)    == LOW;
    _lastBot = digitalRead(PIN_ENDSTOP_BOTTOM) == LOW;
}

void Diagnostics::startJog(bool up, float speedMms) {
    // Delegate the continuous velocity move to MotionController.
    // In JOG mode, update() does nothing — the motor runs until exit() is called.
    _mode = Mode::JOG;
    _mc->jog(up, speedMms);

    Serial.print("DIAG:JOG:");
    Serial.print(up ? "UP:" : "DOWN:");
    Serial.println(speedMms, 1);
}

void Diagnostics::printPosition() {
    Serial.print("DIAG:POS:");
    Serial.println(_mc->getPositionMm(), 2);
}

void Diagnostics::startMoveTest(float mm, float speedMms, float accelMms2) {
    // Store all parameters before calling moveByMm() so they are available
    // to updateMoveTest() even if moveByMm() triggers an immediate ERROR.
    _moveTestDeltaMm   = mm;
    _moveTestSpeedMms  = speedMms;
    _moveTestAccelMms2 = accelMms2;
    _moveTestStartPos  = _mc->getPositionMm();
    _moveTestStart     = millis();
    _moveStarted       = false;
    _mode              = Mode::MOVE_TEST;

    _mc->moveByMm(mm, speedMms, accelMms2);

    // If moveByMm() hit a soft limit, MotionController calls estop() which
    // transitions the state to ERROR.  In that case the test is meaningless —
    // abort silently so the error message from MotionController is the only
    // output (no confusing START/FAIL pair).
    if (_sm->getState() == SystemState::ERROR) {
        _mode = Mode::INACTIVE;
        return;
    }

    // Only print START after confirming the move was accepted.
    Serial.print("DIAG:MOVE:START mm=");
    Serial.print(mm, 1);
    Serial.print(" speed=");
    Serial.print(speedMms, 3);
    Serial.print("mm/s accel=");
    Serial.print(accelMms2, 1);
    Serial.println("mm/s2");
}

void Diagnostics::exit() {
    // Stop the motor if any active mode might be driving it.
    if (_mode == Mode::MOTOR_TEST ||
        _mode == Mode::JOG        ||
        _mode == Mode::MOVE_TEST) {
        _mc->stop();
    }
    _mode = Mode::INACTIVE;
    Serial.println("DIAG:EXIT");
}

void Diagnostics::update() {
    // Fan out to the appropriate handler for the current mode.
    // Only one mode is active at a time, so at most one handler runs per loop.
    if (_mode == Mode::MOTOR_TEST)   updateMotorTest();
    if (_mode == Mode::ENDSTOP_TEST) updateEndstopTest();
    if (_mode == Mode::MOVE_TEST)    updateMoveTest();
    if (_mode == Mode::CAL_MOVE)     updateCalMove();
}

// =============================================================================
// Private helpers
// =============================================================================

void Diagnostics::check(const char* name, bool ok) {
    // Increment the appropriate counter and emit a PASS or FAIL line.
    if (ok) { _passed++; Serial.print("DIAG:PASS:"); }
    else    { _failed++; Serial.print("DIAG:FAIL:"); }
    Serial.println(name);
}

// =============================================================================
// Motor test state machine
// =============================================================================
//
// Phases:
//   MOVE_DOWN  — command DIAG_DIST_MM downward; poll until position settles
//   MOVE_UP    — command DIAG_DIST_MM upward;   poll until position settles
//   DONE       — print summary and deactivate
//
// The remaining MotorPhase values (JOG_DOWN … ACCURACY_UP_CHECK) are reserved
// for a future extended motor characterisation test and are not yet active.
//
// Completion detection uses a position-stability heuristic:
//   The motor is considered done when its position has not moved more than
//   STABLE_MM within the last SETTLE_MS milliseconds.  This works reliably
//   for DIAG_SPEED_MMS (2 mm/s) but would fail at very low speeds — for slow
//   moves use DIAG MOVE instead, which uses the STANDSTILL signal.
//
// =============================================================================

void Diagnostics::updateMotorTest() {
    uint32_t now      = millis();
    uint32_t elapsed  = now - _phaseStart;
    bool     timedOut = elapsed >= MOVE_TIMEOUT_MS;

    switch (_motorPhase) {

        // -----------------------------------------------------------------
        case MotorPhase::MOVE_DOWN: {
            // ---- Timeout guard ------------------------------------------
            if (timedOut) {
                Serial.println("DIAG:MOTOR:FAIL timeout waiting for move down");
                _motorPhase = MotorPhase::DONE;
                break;
            }
            float pos = _mc->getPositionMm();

            // ---- Movement detection -------------------------------------
            // Wait until the motor has left the start position by at least
            // MOVE_START_MM before we start checking for stability.
            // This prevents a false "settled" reading at t=0.
            if (!_moveStarted) {
                if (fabsf(pos - _startPos) >= MOVE_START_MM) {
                    _moveStarted = true;
                    _trackedPos  = pos;
                    _stableSince = now;
                }
                break;
            }

            // ---- Stability tracking -------------------------------------
            // If the position changed significantly, reset the stable timer.
            // Once the position has been stable for SETTLE_MS, the motor
            // is considered to have finished the move.
            if (fabsf(pos - _trackedPos) > STABLE_MM) {
                _trackedPos  = pos;
                _stableSince = now;
            }
            if ((now - _stableSince) < SETTLE_MS) break;

            // ---- Move complete -------------------------------------------
            // Optionally check encoder accuracy, then command the return move.
            if (_checkEncoder) {
                float actual = pos - _startPos;
                check("Move down", fabsf(fabsf(actual) - DIAG_DIST_MM) <= TOLERANCE_MM);
            } else {
                Serial.println("DIAG:MOTOR:move down complete");
            }
            _startPos    = pos;
            _moveStarted = false;
            _phaseStart  = now;
            _motorPhase  = MotorPhase::MOVE_UP;
            _mc->moveByMm(-DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
            break;
        }

        // -----------------------------------------------------------------
        case MotorPhase::MOVE_UP: {
            // Identical structure to MOVE_DOWN — wait, detect movement,
            // track stability, check encoder, then mark DONE.
            if (timedOut) {
                Serial.println("DIAG:MOTOR:FAIL timeout waiting for move up");
                _motorPhase = MotorPhase::DONE;
                break;
            }
            float pos = _mc->getPositionMm();

            if (!_moveStarted) {
                if (fabsf(pos - _startPos) >= MOVE_START_MM) {
                    _moveStarted = true;
                    _trackedPos  = pos;
                    _stableSince = now;
                }
                break;
            }

            if (fabsf(pos - _trackedPos) > STABLE_MM) {
                _trackedPos  = pos;
                _stableSince = now;
            }
            if ((now - _stableSince) < SETTLE_MS) break;

            if (_checkEncoder) {
                float actual = pos - _startPos;
                check("Move up", fabsf(fabsf(actual) - DIAG_DIST_MM) <= TOLERANCE_MM);
            } else {
                Serial.println("DIAG:MOTOR:move up complete");
            }
            _motorPhase = MotorPhase::DONE;
            break;
        }

        // -----------------------------------------------------------------
        case MotorPhase::DONE: {
            // Print final summary (with pass/fail counts for encoder test),
            // then stop the motor and return to INACTIVE.
            const char* tag = _checkEncoder ? "DIAG:MOTORENCODER:DONE" : "DIAG:MOTOR:DONE";
            if (_checkEncoder) {
                Serial.print(tag);
                Serial.print(" passed="); Serial.print(_passed);
                Serial.print(" failed="); Serial.println(_failed);
            } else {
                Serial.println(tag);
            }
            _mc->stop();
            _mode = Mode::INACTIVE;
            break;
        }

        default:
            break;   // extended phases not active yet
    }
}

// =============================================================================
// Move test — command a fixed distance, detect completion via STANDSTILL signal
// =============================================================================
//
// Completion is detected by polling isMoving() (which reads the TMC5130
// STANDSTILL register) rather than a position-stability heuristic.  The
// heuristic approach (used in updateMotorTest) fails below ~0.17 mm/s because
// the motor moves less than STABLE_MM per SETTLE_MS window at that speed.
//
// A dynamic timeout prevents false timeout failures at very slow speeds:
//   timeoutMs = max(MOVE_TIMEOUT_MS,  3 × expected_travel_time)
//
// =============================================================================

void Diagnostics::updateMoveTest() {
    uint32_t now        = millis();
    float    currentPos = _mc->getPositionMm();

    // ---- Dynamic timeout calculation ----------------------------------------
    // At very slow speeds the expected travel time can exceed MOVE_TIMEOUT_MS.
    // Multiply by 3 to allow for acceleration and deceleration phases.
    uint32_t timeoutMs = max(MOVE_TIMEOUT_MS,
                             (uint32_t)((fabsf(_moveTestDeltaMm) / _moveTestSpeedMms) * 3000.0f));
    bool timedOut = (now - _moveTestStart) >= timeoutMs;

    // ---- Movement detection guard -------------------------------------------
    // Ignore STANDSTILL before the motor has visibly left the start position.
    // Without this guard a reading at t=0 (before the motor has started) would
    // appear as "stopped" and immediately end the test with actual=0mm.
    if (!_moveStarted) {
        if (fabsf(currentPos - _moveTestStartPos) >= MOVE_START_MM) {
            _moveStarted = true;
        } else if (!timedOut) {
            return;   // still waiting for motor to start moving
        }
    }

    // ---- Wait for motor to stop ---------------------------------------------
    // isMoving() returns true while the motor is actively stepping (1=moving,
    // 0=stopped).  Stay here until the motor has stopped or the timeout fires.
    if (_mc->isMoving() && !timedOut) return;

    // ---- Evaluate result ----------------------------------------------------
    float actual = currentPos - _moveTestStartPos;
    bool  ok     = fabsf(fabsf(actual) - fabsf(_moveTestDeltaMm)) <= TOLERANCE_MM;

    Serial.print("DIAG:MOVE:");
    Serial.print(ok ? "PASS" : "FAIL");
    Serial.print(" commanded=");
    Serial.print(_moveTestDeltaMm, 1);
    Serial.print("mm actual=");
    Serial.print(actual, 1);
    Serial.println("mm");

    _mc->stop();
    _mode = Mode::INACTIVE;
}

// =============================================================================
// Calibration move — command a distance, measure encoder, report correction
// =============================================================================

void Diagnostics::startCalMove(float mm) {
    // Seed calibration state and start the move at the faster CAL_SPEED_MMS.
    _calCommandedMm = mm;
    _calStartPos    = _mc->getPositionMm();
    _calStart       = millis();
    _calEncoderMm   = 0.0f;
    _moveStarted    = false;
    _mode           = Mode::CAL_MOVE;

    _mc->moveByMm(mm, CAL_SPEED_MMS, DIAG_ACCEL_MMS2);

    Serial.print("DIAG:CAL:START commanded=");
    Serial.print(mm, 1);
    Serial.print("mm at ");
    Serial.print(CAL_SPEED_MMS, 0);
    Serial.println("mm/s");
}

void Diagnostics::updateCalMove() {
    uint32_t now        = millis();
    float    currentPos = _mc->getPositionMm();
    bool     timedOut   = (now - _calStart) >= MOVE_TIMEOUT_MS;

    // ---- Movement detection guard -------------------------------------------
    // Same pattern as updateMotorTest: wait for the motor to visibly leave the
    // start before starting the stability check.
    if (!_moveStarted) {
        if (fabsf(currentPos - _calStartPos) >= MOVE_START_MM) {
            _moveStarted = true;
            _trackedPos  = currentPos;
            _stableSince = now;
        } else if (!timedOut) {
            return;
        }
    }

    // ---- Stability tracking -------------------------------------------------
    // Refresh the stable timer whenever the position moves more than STABLE_MM.
    if (fabsf(currentPos - _trackedPos) > STABLE_MM) {
        _trackedPos  = currentPos;
        _stableSince = now;
    }

    bool settled = (now - _stableSince) >= SETTLE_MS;
    if (!settled && !timedOut) return;

    // ---- Record encoder result ----------------------------------------------
    // The coordinate convention has positive = up.  The sign is flipped here
    // so that _calEncoderMm is positive for a downward (dip) move, which is
    // more natural when comparing to a caliper measurement on the bench.
    float actual  = currentPos - _calStartPos;
    _calEncoderMm = -actual;

    Serial.print("DIAG:CAL:DONE encoder=");
    Serial.print(_calEncoderMm, 2);
    Serial.println("mm");
    Serial.println("DIAG:CAL:Measure the actual displacement with calipers.");
    Serial.println("DIAG:CAL:Then type: DIAG CAL RESULT <actual_mm>");

    _mc->stop();
    _mode = Mode::INACTIVE;
}

void Diagnostics::computeCalResult(float actualMm) {
    // Guard: require a valid calibration move to have been recorded first.
    if (fabsf(_calEncoderMm) < 1.0f) {
        Serial.println("DIAG:CAL:ERR no calibration move recorded - run DIAG CAL <mm> first");
        return;
    }

    // ---- Compute the correction factor ---------------------------------------
    // If the encoder reports X mm but the physical displacement is Y mm, the
    // leadscrew pitch constant (LEADSCREW_MM_PER_REV) is off by a factor of Y/X.
    // Multiplying the current constant by this factor gives the corrected value.
    float correction  = actualMm / _calEncoderMm;
    float newMmPerRev = LEADSCREW_MM_PER_REV * correction;

    // ---- Print the result ---------------------------------------------------
    Serial.print("DIAG:CAL:RESULT commanded=");
    Serial.print(_calCommandedMm, 1);
    Serial.print("mm encoder=");
    Serial.print(_calEncoderMm, 2);
    Serial.print("mm actual=");
    Serial.print(actualMm, 2);
    Serial.println("mm");

    Serial.print("DIAG:CAL:correction=");
    Serial.print(correction, 4);
    if (fabsf(correction - 1.0f) < 0.01f) {
        // Within 1% — no change to config.h needed.
        Serial.println(" (within 1% - no change needed)");
    } else {
        // Print the new value so the user can paste it directly into config.h.
        Serial.println();
        Serial.print("DIAG:CAL:Update config.h: #define LEADSCREW_MM_PER_REV  ");
        Serial.print(newMmPerRev, 4);
        Serial.println("f");
    }
}

// =============================================================================
// Endstop test — poll with debounce, print on state change
// =============================================================================

void Diagnostics::updateEndstopTest() {
    // Static variables persist across calls but are local to this function.
    // They track debounce state independently for each endstop.
    static uint32_t topDebounceStart = 0;
    static uint32_t botDebounceStart = 0;
    static bool     pendingTop       = false;
    static bool     pendingBot       = false;
    static bool     pendingTopVal    = false;
    static bool     pendingBotVal    = false;

    uint32_t now = millis();

    // Read raw pin states.  Endstops are active-LOW with INPUT_PULLUP,
    // so LOW = triggered.
    bool rawTop = digitalRead(PIN_ENDSTOP_TOP)    == LOW;
    bool rawBot = digitalRead(PIN_ENDSTOP_BOTTOM) == LOW;

    // ---- Top endstop debounce -----------------------------------------------
    if (rawTop != _lastTop) {
        if (!pendingTop) {
            // New transition detected — start the debounce timer.
            pendingTop       = true;
            pendingTopVal    = rawTop;
            topDebounceStart = now;
        } else if (rawTop == pendingTopVal && (now - topDebounceStart) >= DEBOUNCE_MS) {
            // The transition has been stable for DEBOUNCE_MS — accept it.
            _lastTop   = rawTop;
            pendingTop = false;
            Serial.print("DIAG:ENDSTOP:TOP:");
            Serial.println(rawTop ? "TRIGGERED" : "RELEASED");
        }
    } else {
        // Input returned to its previous state before the debounce window ended
        // — it was noise, discard the pending transition.
        pendingTop = false;
    }

    // ---- Bottom endstop debounce --------------------------------------------
    if (rawBot != _lastBot) {
        if (!pendingBot) {
            pendingBot       = true;
            pendingBotVal    = rawBot;
            botDebounceStart = now;
        } else if (rawBot == pendingBotVal && (now - botDebounceStart) >= DEBOUNCE_MS) {
            _lastBot   = rawBot;
            pendingBot = false;
            Serial.print("DIAG:ENDSTOP:BOTTOM:");
            Serial.println(rawBot ? "TRIGGERED" : "RELEASED");
        }
    } else {
        pendingBot = false;
    }
}
