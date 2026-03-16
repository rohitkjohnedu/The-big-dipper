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
{}

void Diagnostics::begin(MotionController& mc, StateMachine& sm) {
    _mc = &mc;
    _sm = &sm;
}

// =============================================================================
// Public interface
// =============================================================================

void Diagnostics::startMotorTest() {
    Serial.println("DIAG:MOTOR:START");
    _mode       = Mode::MOTOR_TEST;
    _motorPhase = MotorPhase::JOG_DOWN;
    _passed     = 0;
    _failed     = 0;
    _startPos   = _mc->getPositionMm();   // capture pre-jog position
    _phaseStart = millis();
    _mc->jog(false, JOG_SPEED);           // false = down
}

void Diagnostics::startEndstopTest() {
    Serial.println("DIAG:ENDSTOP:START");
    Serial.println("DIAG:ENDSTOP:Trigger each endstop manually. Send DIAG EXIT to stop.");
    _mode    = Mode::ENDSTOP_TEST;
    _lastTop = digitalRead(PIN_ENDSTOP_TOP)    == LOW;
    _lastBot = digitalRead(PIN_ENDSTOP_BOTTOM) == LOW;
}

void Diagnostics::exit() {
    if (_mode == Mode::MOTOR_TEST) {
        _mc->stop();
    }
    _mode = Mode::INACTIVE;
    Serial.println("DIAG:EXIT");
}

void Diagnostics::update() {
    if (_mode == Mode::MOTOR_TEST)   updateMotorTest();
    if (_mode == Mode::ENDSTOP_TEST) updateEndstopTest();
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

    switch (_motorPhase) {

        // ----------------------------------------------------------------
        // Jog down: verify motor and encoder are present
        // ----------------------------------------------------------------
        case MotorPhase::JOG_DOWN:
            if (elapsed >= PRESENCE_MS) {
                _mc->stop();
                _phaseStart = now;
                _motorPhase = MotorPhase::JOG_DOWN_SETTLE;
            }
            break;

        case MotorPhase::JOG_DOWN_SETTLE:
            if (elapsed >= SETTLE_MS) {
                _motorPhase = MotorPhase::JOG_DOWN_CHECK;
            }
            break;

        case MotorPhase::JOG_DOWN_CHECK: {
            float moved = _mc->getPositionMm() - _startPos;
            check("Motor present (encoder moved >= 3mm)", moved >= PRESENCE_MIN_MM);
            // Transition state machine to HOMING so HOME_WAIT can detect completion
            _sm->toHoming();
            _mc->executeHome();
            _phaseStart = now;
            _motorPhase = MotorPhase::HOME_WAIT;
            break;
        }

        // ----------------------------------------------------------------
        // Homing: drive up until top endstop fires, then back off
        // ----------------------------------------------------------------
        case MotorPhase::HOME_WAIT:
            if (_sm->getState() == SystemState::READY) {
                _motorPhase = MotorPhase::HOME_CHECK;
            } else if (elapsed >= HOME_TIMEOUT_MS) {
                check("Homing completed within timeout", false);
                _motorPhase = MotorPhase::DONE;
            }
            break;

        case MotorPhase::HOME_CHECK:
            check("Homing completed within timeout", true);
            _startPos   = _mc->getPositionMm();
            _mc->jog(false, JOG_SPEED);   // down
            _phaseStart = now;
            _motorPhase = MotorPhase::ACCURACY_DOWN;
            break;

        // ----------------------------------------------------------------
        // Accuracy down
        // ----------------------------------------------------------------
        case MotorPhase::ACCURACY_DOWN:
            if (elapsed >= ACCURACY_DOWN_MS) {
                _mc->stop();
                _phaseStart = now;
                _motorPhase = MotorPhase::ACCURACY_DOWN_SETTLE;
            }
            break;

        case MotorPhase::ACCURACY_DOWN_SETTLE:
            if (elapsed >= SETTLE_MS) {
                _motorPhase = MotorPhase::ACCURACY_DOWN_CHECK;
            }
            break;

        case MotorPhase::ACCURACY_DOWN_CHECK: {
            float expected = JOG_SPEED * (ACCURACY_DOWN_MS / 1000.0f);
            float actual   = _mc->getPositionMm() - _startPos;
            check("Accuracy down (within 2mm)", fabsf(actual - expected) <= TOLERANCE_MM);
            _startPos   = _mc->getPositionMm();
            _mc->jog(true, JOG_SPEED);    // up
            _phaseStart = now;
            _motorPhase = MotorPhase::ACCURACY_UP;
            break;
        }

        // ----------------------------------------------------------------
        // Accuracy up
        // ----------------------------------------------------------------
        case MotorPhase::ACCURACY_UP:
            if (elapsed >= ACCURACY_UP_MS) {
                _mc->stop();
                _phaseStart = now;
                _motorPhase = MotorPhase::ACCURACY_UP_SETTLE;
            }
            break;

        case MotorPhase::ACCURACY_UP_SETTLE:
            if (elapsed >= SETTLE_MS) {
                _motorPhase = MotorPhase::ACCURACY_UP_CHECK;
            }
            break;

        case MotorPhase::ACCURACY_UP_CHECK: {
            float expected = JOG_SPEED * (ACCURACY_UP_MS / 1000.0f);
            float actual   = _startPos - _mc->getPositionMm();   // up = decrease in mm
            check("Accuracy up (within 2mm)", fabsf(actual - expected) <= TOLERANCE_MM);
            _motorPhase = MotorPhase::DONE;
            break;
        }

        // ----------------------------------------------------------------
        // Done
        // ----------------------------------------------------------------
        case MotorPhase::DONE:
            Serial.print("DIAG:MOTOR:DONE passed=");
            Serial.print(_passed);
            Serial.print(" failed=");
            Serial.println(_failed);
            _mode = Mode::INACTIVE;
            break;
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
