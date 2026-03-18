#include "diagnostics.h"

// =============================================================================
// Constructor / begin
// =============================================================================

Diagnostics::Diagnostics()
    : _mc(nullptr), _sm(nullptr)
    , _mode(Mode::INACTIVE)
    , _motorPhase(MotorPhase::JOG_DOWN)
    , _phaseStart(0), _startPos(0.0f)
    , _passed(0), _failed(0)
    , _lastTop(false), _lastBot(false)
    , _moveTestStartPos(0.0f), _moveTestDeltaMm(0.0f), _moveTestSpeedMms(DIAG_SPEED_MMS), _moveTestAccelMms2(DIAG_ACCEL_MMS2), _moveTestStart(0)
    , _moveStarted(false), _trackedPos(0.0f), _stableSince(0)
    , _checkEncoder(false)
    , _calStartPos(0.0f), _calCommandedMm(0.0f), _calEncoderMm(0.0f), _calStart(0)
{}

void Diagnostics::begin(MotionController& mc, StateMachine& sm) {
    _mc = &mc;
    _sm = &sm;
}

// =============================================================================
// Public interface
// =============================================================================

void Diagnostics::startMotorTest() {
    Serial.print("DIAG:MOTOR:START down=");
    Serial.print(DIAG_DIST_MM, 0);
    Serial.print("mm speed=");
    Serial.print(DIAG_SPEED_MMS, 0);
    Serial.println("mm/s");
    _mode          = Mode::MOTOR_TEST;
    _motorPhase    = MotorPhase::MOVE_DOWN;
    _passed        = 0;
    _failed        = 0;
    _checkEncoder  = false;
    _startPos      = _mc->getPositionMm();
    _phaseStart    = millis();
    _moveStarted = false;
    _mc->moveByMm(-DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);  // user convention: positive=up, so -dist moves down
}

void Diagnostics::startMotorEncoderTest() {
    Serial.print("DIAG:MOTORENCODER:START down=");
    Serial.print(DIAG_DIST_MM, 0);
    Serial.print("mm speed=");
    Serial.print(DIAG_SPEED_MMS, 0);
    Serial.println("mm/s");
    _mode          = Mode::MOTOR_TEST;
    _motorPhase    = MotorPhase::MOVE_DOWN;
    _passed        = 0;
    _failed        = 0;
    _checkEncoder  = true;
    _startPos      = _mc->getPositionMm();
    _phaseStart    = millis();
    _moveStarted   = false;
    _mc->moveByMm(-DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);  // user convention: positive=up, so -dist moves down
}

void Diagnostics::startEndstopTest() {
    Serial.println("DIAG:ENDSTOP:START");
    Serial.println("DIAG:ENDSTOP:Trigger each endstop manually. Send DIAG EXIT to stop.");
    _mode    = Mode::ENDSTOP_TEST;
    _lastTop = digitalRead(PIN_ENDSTOP_TOP)    == LOW;
    _lastBot = digitalRead(PIN_ENDSTOP_BOTTOM) == LOW;
}

void Diagnostics::startJog(bool up, float speedMms) {
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
    _moveTestDeltaMm   = mm;
    _moveTestSpeedMms  = speedMms;
    _moveTestAccelMms2 = accelMms2;
    _moveTestStartPos  = _mc->getPositionMm();
    _moveTestStart     = millis();
    _moveStarted       = false;
    _mode              = Mode::MOVE_TEST;
    _mc->moveByMm(-mm, speedMms, accelMms2);  // negate: positive mm = up (toward home)
    Serial.print("DIAG:MOVE:START mm=");
    Serial.print(mm, 1);
    Serial.print(" speed=");
    Serial.print(speedMms, 1);
    Serial.print("mm/s accel=");
    Serial.print(accelMms2, 1);
    Serial.println("mm/s2");
}

void Diagnostics::exit() {
    if (_mode == Mode::MOTOR_TEST || _mode == Mode::JOG || _mode == Mode::MOVE_TEST) {
        _mc->stop();
    }
    _mode = Mode::INACTIVE;
    Serial.println("DIAG:EXIT");
}

void Diagnostics::update() {
    if (_mode == Mode::MOTOR_TEST)   updateMotorTest();
    if (_mode == Mode::ENDSTOP_TEST) updateEndstopTest();
    if (_mode == Mode::MOVE_TEST)    updateMoveTest();
    if (_mode == Mode::CAL_MOVE)     updateCalMove();
}

// =============================================================================
// Private helpers
// =============================================================================

void Diagnostics::check(const char* name, bool ok) {
    if (ok) {
        _passed++;
        Serial.print("DIAG:PASS:");
    } else {
        _failed++;
        Serial.print("DIAG:FAIL:");
    }
    Serial.println(name);
}

// =============================================================================
// Motor test state machine
// =============================================================================
//
// Phases:
//  JOG_DOWN            — jog down for PRESENCE_MS
//  JOG_DOWN_SETTLE     — wait SETTLE_MS for encoder to stabilise
//  JOG_DOWN_CHECK      — verify encoder moved >= PRESENCE_MIN_MM; start homing
//  HOME_WAIT           — wait for state machine to reach READY (homing done)
//  HOME_CHECK          — emit pass/fail; jog down for accuracy test
//  ACCURACY_DOWN       — jog down for ACCURACY_DOWN_MS
//  ACCURACY_DOWN_SETTLE— wait SETTLE_MS
//  ACCURACY_DOWN_CHECK — compare encoder vs commanded distance
//  ACCURACY_UP         — jog up for ACCURACY_UP_MS
//  ACCURACY_UP_SETTLE  — wait SETTLE_MS
//  ACCURACY_UP_CHECK   — compare encoder vs commanded distance
//  DONE                — print summary, deactivate
//
// =============================================================================

void Diagnostics::updateMotorTest() {
    uint32_t now      = millis();
    uint32_t elapsed  = now - _phaseStart;
    bool     timedOut = elapsed >= MOVE_TIMEOUT_MS;

    switch (_motorPhase) {

        case MotorPhase::MOVE_DOWN: {
            if (timedOut) {
                Serial.println("DIAG:MOTOR:FAIL timeout waiting for move down");
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

            // Motor has settled — check encoder and start return move
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
            _mc->moveByMm(DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);  // user convention: positive=up, so +dist moves up
            break;
        }

        case MotorPhase::MOVE_UP: {
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

            // Motor has settled — check encoder
            if (_checkEncoder) {
                float actual = pos - _startPos;
                check("Move up", fabsf(fabsf(actual) - DIAG_DIST_MM) <= TOLERANCE_MM);
            } else {
                Serial.println("DIAG:MOTOR:move up complete");
            }
            _motorPhase = MotorPhase::DONE;
            break;
        }

        case MotorPhase::DONE: {
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
// Move test — command a fixed distance, detect stop via position stability
// =============================================================================

void Diagnostics::updateMoveTest() {
    uint32_t now        = millis();
    float    currentPos = _mc->getPositionMm();
    bool     timedOut   = (now - _moveTestStart) >= MOVE_TIMEOUT_MS;

    if (!_moveStarted) {
        // Wait until the motor has actually left the start position before
        // checking for stability — avoids a false "done" at t=0.
        if (fabsf(currentPos - _moveTestStartPos) >= MOVE_START_MM) {
            _moveStarted = true;
            _trackedPos  = currentPos;
            _stableSince = now;
        } else if (!timedOut) {
            return;
        }
    }

    // If position moved significantly, reset the stability timer.
    if (fabsf(currentPos - _trackedPos) > STABLE_MM) {
        _trackedPos  = currentPos;
        _stableSince = now;
    }

    bool settled = (now - _stableSince) >= SETTLE_MS;
    if (!settled && !timedOut) return;

    float actual        = currentPos - _moveTestStartPos;
    float displayActual = actual;   // positive = up, matches user-facing convention

    bool ok = fabsf(fabsf(actual) - fabsf(_moveTestDeltaMm)) <= TOLERANCE_MM;
    Serial.print("DIAG:MOVE:");
    Serial.print(ok ? "PASS" : "FAIL");
    Serial.print(" commanded=");
    Serial.print(_moveTestDeltaMm, 1);
    Serial.print("mm actual=");
    Serial.print(displayActual, 1);
    Serial.println("mm");

    _mc->stop();
    _mode = Mode::INACTIVE;
}

// =============================================================================
// Calibration move — command a distance, measure encoder, report correction
// =============================================================================

void Diagnostics::startCalMove(float mm) {
    _calCommandedMm = mm;
    _calStartPos    = _mc->getPositionMm();
    _calStart       = millis();
    _calEncoderMm   = 0.0f;
    _moveStarted    = false;
    _mode           = Mode::CAL_MOVE;
    _mc->moveByMm(-mm, CAL_SPEED_MMS, DIAG_ACCEL_MMS2);  // user convention: positive=up, negate to match firmware
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

    if (!_moveStarted) {
        if (fabsf(currentPos - _calStartPos) >= MOVE_START_MM) {
            _moveStarted = true;
            _trackedPos  = currentPos;
            _stableSince = now;
        } else if (!timedOut) {
            return;
        }
    }

    if (fabsf(currentPos - _trackedPos) > STABLE_MM) {
        _trackedPos  = currentPos;
        _stableSince = now;
    }

    bool settled = (now - _stableSince) >= SETTLE_MS;
    if (!settled && !timedOut) return;

    float actual  = currentPos - _calStartPos;
    _calEncoderMm = -actual;   // positive = down (matches physical measurement direction)

    Serial.print("DIAG:CAL:DONE encoder=");
    Serial.print(_calEncoderMm, 2);
    Serial.println("mm");
    Serial.println("DIAG:CAL:Measure the actual displacement with calipers.");
    Serial.println("DIAG:CAL:Then type: DIAG CAL RESULT <actual_mm>");

    _mc->stop();
    _mode = Mode::INACTIVE;
}

void Diagnostics::computeCalResult(float actualMm) {
    if (fabsf(_calEncoderMm) < 1.0f) {
        Serial.println("DIAG:CAL:ERR no calibration move recorded - run DIAG CAL <mm> first");
        return;
    }
    // If encoder reads X mm but physical displacement is Y mm, the leadscrew
    // pitch constant is off by a factor of Y/X. Scale LEADSCREW_MM_PER_REV accordingly.
    float correction  = actualMm / _calEncoderMm;
    float newMmPerRev = LEADSCREW_MM_PER_REV * correction;

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
        Serial.println(" (within 1% - no change needed)");
    } else {
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
    static uint32_t topDebounceStart = 0;
    static uint32_t botDebounceStart = 0;
    static bool     pendingTop       = false;
    static bool     pendingBot       = false;
    static bool     pendingTopVal    = false;
    static bool     pendingBotVal    = false;

    uint32_t now = millis();

    bool rawTop = digitalRead(PIN_ENDSTOP_TOP)    == LOW;
    bool rawBot = digitalRead(PIN_ENDSTOP_BOTTOM) == LOW;

    // Top endstop
    if (rawTop != _lastTop) {
        if (!pendingTop) {
            pendingTop       = true;
            pendingTopVal    = rawTop;
            topDebounceStart = now;
        } else if (rawTop == pendingTopVal && (now - topDebounceStart) >= DEBOUNCE_MS) {
            _lastTop   = rawTop;
            pendingTop = false;
            Serial.print("DIAG:ENDSTOP:TOP:");
            Serial.println(rawTop ? "TRIGGERED" : "RELEASED");
        }
    } else {
        pendingTop = false;
    }

    // Bottom endstop
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
