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
    , _moveTestStartPos(0.0f), _moveTestDeltaMm(0.0f), _moveTestStart(0)
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
    _mc->moveByMm(DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
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
    _mc->moveByMm(DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
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

void Diagnostics::startMoveTest(float mm) {
    _moveTestDeltaMm  = mm;
    _moveTestStartPos = _mc->getPositionMm();
    _moveTestStart    = millis();
    _mode             = Mode::MOVE_TEST;
    _mc->moveByMm(mm, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
    Serial.print("DIAG:MOVE:START mm=");
    Serial.println(mm, 1);
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
    uint32_t now     = millis();
    uint32_t elapsed = now - _phaseStart;
    bool     timedOut = elapsed >= MOVE_TIMEOUT_MS;

    switch (_motorPhase) {

        case MotorPhase::MOVE_DOWN:
            if (timedOut) {
                Serial.println("DIAG:MOTOR:FAIL timeout waiting for move down");
                _motorPhase = MotorPhase::DONE;
                break;
            }
            if (_mc->isMoveDone()) {
                _phaseStart = now;
                _motorPhase = MotorPhase::MOVE_DOWN_WAIT;
            }
            break;

        case MotorPhase::MOVE_DOWN_WAIT:
            if (elapsed >= SETTLE_MS) {
                if (_checkEncoder) {
                    float actual = _mc->getPositionMm() - _startPos;
                    check("Move down (encoder within tolerance)",
                          fabsf(fabsf(actual) - DIAG_DIST_MM) <= TOLERANCE_MM);
                } else {
                    Serial.println("DIAG:MOTOR:move down complete");
                }
                _startPos   = _mc->getPositionMm();
                _phaseStart = now;
                _motorPhase = MotorPhase::MOVE_UP;
                _mc->moveByMm(-DIAG_DIST_MM, DIAG_SPEED_MMS, DIAG_ACCEL_MMS2);
            }
            break;

        case MotorPhase::MOVE_UP:
            if (timedOut) {
                Serial.println("DIAG:MOTOR:FAIL timeout waiting for move up");
                _motorPhase = MotorPhase::DONE;
                break;
            }
            if (_mc->isMoveDone()) {
                _phaseStart = now;
                _motorPhase = MotorPhase::MOVE_UP_WAIT;
            }
            break;

        case MotorPhase::MOVE_UP_WAIT:
            if (elapsed >= SETTLE_MS) {
                if (_checkEncoder) {
                    float actual = _startPos - _mc->getPositionMm();
                    check("Move up (encoder within tolerance)",
                          fabsf(fabsf(actual) - DIAG_DIST_MM) <= TOLERANCE_MM);
                } else {
                    Serial.println("DIAG:MOTOR:move up complete");
                }
                _motorPhase = MotorPhase::DONE;
            }
            break;

        case MotorPhase::DONE: {
            const char* tag = _checkEncoder ? "DIAG:MOTORENCODER:DONE" : "DIAG:MOTOR:DONE";
            if (_checkEncoder) {
                Serial.print(tag);
                Serial.print(" passed="); Serial.print(_passed);
                Serial.print(" failed="); Serial.println(_failed);
            } else {
                Serial.println(tag);
            }
            _mode = Mode::INACTIVE;
            break;
        }

        default:
            break;   // extended phases not active yet
    }
}

// =============================================================================
// Move test — command a fixed distance, wait for standstill, check encoder
// =============================================================================

void Diagnostics::updateMoveTest() {
    uint32_t now     = millis();
    uint32_t elapsed = now - _moveTestStart;

    // getMotorState(STANDSTILL) is only briefly true after the motor stops and
    // is unreliable for detecting move completion. Instead, wait the expected
    // move duration then sample the encoder directly.
    uint32_t expectedMs = (uint32_t)(fabsf(_moveTestDeltaMm) / DIAG_SPEED_MMS * 1000.0f) + 500UL;
    if (elapsed < expectedMs) return;

    // Encoder convention: negative = down. Negate for display (positive = down).
    float actual        = _mc->getPositionMm() - _moveTestStartPos;
    float displayActual = -actual;

    bool ok = fabsf(fabsf(actual) - fabsf(_moveTestDeltaMm)) <= TOLERANCE_MM;
    Serial.print("DIAG:MOVE:");
    Serial.print(ok ? "PASS" : "FAIL");
    Serial.print(" commanded=");
    Serial.print(_moveTestDeltaMm, 1);
    Serial.print("mm actual=");
    Serial.print(displayActual, 1);
    Serial.println("mm");

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
    _mode           = Mode::CAL_MOVE;
    _mc->moveByMm(mm, CAL_SPEED_MMS, DIAG_ACCEL_MMS2);
    Serial.print("DIAG:CAL:START commanded=");
    Serial.print(mm, 1);
    Serial.print("mm at ");
    Serial.print(CAL_SPEED_MMS, 0);
    Serial.println("mm/s");
}

void Diagnostics::updateCalMove() {
    uint32_t now     = millis();
    uint32_t elapsed = now - _calStart;

    uint32_t expectedMs = (uint32_t)(fabsf(_calCommandedMm) / CAL_SPEED_MMS * 1000.0f) + 1000UL;
    if (elapsed < expectedMs) return;

    float actual  = _mc->getPositionMm() - _calStartPos;
    _calEncoderMm = -actual;   // positive = down (matches physical measurement direction)

    Serial.print("DIAG:CAL:DONE encoder=");
    Serial.print(_calEncoderMm, 2);
    Serial.println("mm");
    Serial.println("DIAG:CAL:Measure the actual displacement with calipers.");
    Serial.println("DIAG:CAL:Then type: DIAG CAL RESULT <actual_mm>");

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
