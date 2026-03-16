// =============================================================================
// hw_bringup.cpp — Hardware bringup and validation test
//
// Flash:   pio run -e hw_test -t upload
// Monitor: pio device monitor -e hw_test   (115200 baud)
//
// Test sequence
// ─────────────
//   TEST 1 — Motor Connectivity
//     Jogs DOWN at JOG_SPEED_MMS for PRESENCE_JOG_MS.
//     PASS: encoder displacement >= PRESENCE_MIN_DIST_MM.
//     If position goes the wrong direction, swap CW/CCW in jog() / executeHome().
//
//   TEST 2 — Homing
//     Drives UP to top endstop, zeros encoder, backs off HOMING_BACKOFF_MM down.
//     PASS: state == READY, isHomed(), position ≈ HOMING_BACKOFF_MM.
//
//   TEST 3 — Move Accuracy (Down)
//     Jogs DOWN at JOG_SPEED_MMS for ACCURACY_JOG_DOWN_MS.
//     Expected displacement = speed × time.
//     PASS: |actual − expected| <= POSITION_TOLERANCE_MM.
//
//   TEST 4 — Move Accuracy (Up)
//     Jogs UP at JOG_SPEED_MMS for ACCURACY_JOG_UP_MS.
//     Expected displacement = -(speed × time).
//     PASS: |actual − expected| <= POSITION_TOLERANCE_MM.
//
// All results printed to Serial.  Motor halts after final result.
// =============================================================================

#include <Arduino.h>
#include "state_machine.h"
#include "motion_controller.h"
#include "config.h"

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------

static StateMachine     sm;
static MotionController mc(sm);

// ---------------------------------------------------------------------------
// Endstop ISR wrappers
// ---------------------------------------------------------------------------

void onBottomEndstop() { mc.onEndstopTriggered(false); }
void onTopEndstop()    { mc.onEndstopTriggered(true);  }

// ---------------------------------------------------------------------------
// Test parameters  (adjust these to suit travel range / motor speed)
// ---------------------------------------------------------------------------

static constexpr float    JOG_SPEED_MMS         =  5.0f;  // mm/s for all jog tests
static constexpr uint32_t PRESENCE_JOG_MS       =  2000;  // 2 s  →  ~10 mm at 5 mm/s
static constexpr float    PRESENCE_MIN_DIST_MM  =  3.0f;  // motor must move at least this
static constexpr uint32_t HOME_TIMEOUT_MS       = 30000;  // abort homing after 30 s
static constexpr uint32_t ACCURACY_JOG_DOWN_MS  = 10000;  // 10 s →  50 mm at 5 mm/s
static constexpr uint32_t ACCURACY_JOG_UP_MS    =  5000;  //  5 s →  25 mm at 5 mm/s
static constexpr float    POSITION_TOLERANCE_MM =  2.0f;  // ±2 mm acceptable error
static constexpr uint32_t SETTLE_MS             =   500;  // wait after stop() for encoder

// ---------------------------------------------------------------------------
// Test result tracking
// ---------------------------------------------------------------------------

static int _passed = 0, _failed = 0;

static void printCheck(const char* name, bool ok) {
    Serial.print(ok ? "  [PASS] " : "  [FAIL] ");
    Serial.println(name);
    ok ? _passed++ : _failed++;
}

// ---------------------------------------------------------------------------
// Non-blocking phase sequencer
// ---------------------------------------------------------------------------

enum class Phase : uint8_t {
    PRESENCE_JOG,       // jog down, wait for timer or endstop
    PRESENCE_SETTLE,    // wait for motor to stop and encoder to settle
    PRESENCE_CHECK,     // evaluate displacement, issue home command

    HOME_WAIT,          // wait for state == READY or timeout
    HOME_CHECK,         // evaluate homing result

    DOWN_JOG,           // jog down for timed accuracy test
    DOWN_SETTLE,
    DOWN_CHECK,         // evaluate downward displacement

    UP_JOG,             // jog up for timed accuracy test
    UP_SETTLE,
    UP_CHECK,           // evaluate upward displacement

    DONE                // print summary, halt
};

static Phase    phase      = Phase::PRESENCE_JOG;
static uint32_t phaseStart = 0;
static float    startPos   = 0.0f;

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) { delay(10); }  // wait until host opens the port

    Serial.println(F("================================================"));
    Serial.println(F("  Dip Coater — Hardware Bringup Test"));
    Serial.println(F("================================================"));
    Serial.println();

    // Configure endstop pins (active-LOW, hardware pull-up assumed on board)
    pinMode(PIN_ENDSTOP_BOTTOM, INPUT_PULLUP);
    pinMode(PIN_ENDSTOP_TOP,    INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_BOTTOM),
                    onBottomEndstop, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_TOP),
                    onTopEndstop,    FALLING);

    mc.begin();
    Serial.println(F("Motor driver initialised."));
    Serial.println();

    Serial.println(F("--- TEST 1: Motor Connectivity ---"));
    Serial.print(F("  Jogging DOWN at ")); Serial.print(JOG_SPEED_MMS, 1);
    Serial.print(F(" mm/s for "));         Serial.print(PRESENCE_JOG_MS / 1000);
    Serial.println(F(" s."));
    Serial.print(F("  PASS criterion: encoder moves >= "));
    Serial.print(PRESENCE_MIN_DIST_MM, 1); Serial.println(F(" mm."));

    startPos = mc.getPositionMm();
    mc.jog(false, JOG_SPEED_MMS);   // false = down (positive mm direction)
    phaseStart = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------

void loop() {
    mc.update();

    switch (phase) {

    // -----------------------------------------------------------------------
    case Phase::PRESENCE_JOG:
        // End condition: timer elapsed OR unexpected endstop (error state)
        if (millis() - phaseStart >= PRESENCE_JOG_MS ||
            sm.getState() == SystemState::ERROR) {
            mc.stop();
            phaseStart = millis();
            phase = Phase::PRESENCE_SETTLE;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::PRESENCE_SETTLE:
        if (millis() - phaseStart >= SETTLE_MS) {
            phase = Phase::PRESENCE_CHECK;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::PRESENCE_CHECK: {
        float disp = mc.getPositionMm() - startPos;
        Serial.print(F("  Displacement: ")); Serial.print(disp, 2); Serial.println(F(" mm"));
        Serial.println(F("  (positive = DOWN, negative = UP — check wiring if inverted)"));
        printCheck("Motor connected and encoder responding (>= 3 mm moved)",
                   fabsf(disp) >= PRESENCE_MIN_DIST_MM);

        // Recover to a state that allows toHoming() — works from READY or ERROR
        sm.toHoming();

        Serial.println();
        Serial.println(F("--- TEST 2: Homing ---"));
        Serial.println(F("  Motor should drive UP, hit top endstop, then back off."));

        mc.executeHome();
        phaseStart = millis();
        phase = Phase::HOME_WAIT;
        break;
    }

    // -----------------------------------------------------------------------
    case Phase::HOME_WAIT:
        if (sm.getState() == SystemState::READY) {
            phase = Phase::HOME_CHECK;
        } else if (millis() - phaseStart >= HOME_TIMEOUT_MS) {
            Serial.println(F("  TIMEOUT — homing did not complete within 30 s."));
            mc.estop();
            phase = Phase::HOME_CHECK;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::HOME_CHECK: {
        bool homed  = sm.isHomed() && sm.getState() == SystemState::READY;
        float pos   = mc.getPositionMm();
        float offErr = fabsf(pos - HOMING_BACKOFF_MM);

        Serial.print(F("  Position after home: ")); Serial.print(pos, 2); Serial.println(F(" mm"));
        Serial.print(F("  Expected           : ")); Serial.print(HOMING_BACKOFF_MM, 2);
        Serial.println(F(" mm  (= HOMING_BACKOFF_MM)"));

        printCheck("Homing sequence complete (state = READY, isHomed = true)", homed);
        printCheck("Post-home position within 2 mm of backoff offset", offErr <= POSITION_TOLERANCE_MM);

        if (!homed) {
            Serial.println();
            Serial.println(F("  Homing failed — skipping accuracy tests."));
            phase = Phase::DONE;
            break;
        }

        Serial.println();
        Serial.println(F("--- TEST 3: Move Accuracy (Down) ---"));
        float expectedDown = JOG_SPEED_MMS * (ACCURACY_JOG_DOWN_MS / 1000.0f);
        Serial.print(F("  Jogging DOWN ")); Serial.print(expectedDown, 1);
        Serial.print(F(" mm  ("));          Serial.print(JOG_SPEED_MMS, 1);
        Serial.print(F(" mm/s x "));        Serial.print(ACCURACY_JOG_DOWN_MS / 1000);
        Serial.println(F(" s)"));

        startPos = mc.getPositionMm();
        mc.jog(false, JOG_SPEED_MMS);
        phaseStart = millis();
        phase = Phase::DOWN_JOG;
        break;
    }

    // -----------------------------------------------------------------------
    case Phase::DOWN_JOG:
        if (millis() - phaseStart >= ACCURACY_JOG_DOWN_MS ||
            sm.getState() == SystemState::ERROR) {
            mc.stop();
            phaseStart = millis();
            phase = Phase::DOWN_SETTLE;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::DOWN_SETTLE:
        if (millis() - phaseStart >= SETTLE_MS) {
            phase = Phase::DOWN_CHECK;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::DOWN_CHECK: {
        float expected = JOG_SPEED_MMS * (ACCURACY_JOG_DOWN_MS / 1000.0f);
        float actual   = mc.getPositionMm() - startPos;
        float err      = fabsf(actual - expected);

        Serial.print(F("  Expected: ")); Serial.print(expected, 2); Serial.println(F(" mm"));
        Serial.print(F("  Actual  : ")); Serial.print(actual,   2); Serial.println(F(" mm"));
        Serial.print(F("  Error   : ")); Serial.print(err,      2); Serial.println(F(" mm"));
        printCheck("Down displacement within 2 mm of commanded", err <= POSITION_TOLERANCE_MM);

        Serial.println();
        Serial.println(F("--- TEST 4: Move Accuracy (Up) ---"));
        float expectedUp = JOG_SPEED_MMS * (ACCURACY_JOG_UP_MS / 1000.0f);
        Serial.print(F("  Jogging UP ")); Serial.print(expectedUp, 1);
        Serial.print(F(" mm  ("));        Serial.print(JOG_SPEED_MMS, 1);
        Serial.print(F(" mm/s x "));      Serial.print(ACCURACY_JOG_UP_MS / 1000);
        Serial.println(F(" s)"));

        startPos = mc.getPositionMm();
        mc.jog(true, JOG_SPEED_MMS);   // true = up (negative mm direction)
        phaseStart = millis();
        phase = Phase::UP_JOG;
        break;
    }

    // -----------------------------------------------------------------------
    case Phase::UP_JOG:
        if (millis() - phaseStart >= ACCURACY_JOG_UP_MS ||
            sm.getState() == SystemState::ERROR) {
            mc.stop();
            phaseStart = millis();
            phase = Phase::UP_SETTLE;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::UP_SETTLE:
        if (millis() - phaseStart >= SETTLE_MS) {
            phase = Phase::UP_CHECK;
        }
        break;

    // -----------------------------------------------------------------------
    case Phase::UP_CHECK: {
        float expected = -(JOG_SPEED_MMS * (ACCURACY_JOG_UP_MS / 1000.0f)); // negative = up
        float actual   = mc.getPositionMm() - startPos;
        float err      = fabsf(actual - expected);

        Serial.print(F("  Expected: ")); Serial.print(expected, 2); Serial.println(F(" mm"));
        Serial.print(F("  Actual  : ")); Serial.print(actual,   2); Serial.println(F(" mm"));
        Serial.print(F("  Error   : ")); Serial.print(err,      2); Serial.println(F(" mm"));
        printCheck("Up displacement within 2 mm of commanded", err <= POSITION_TOLERANCE_MM);

        phase = Phase::DONE;
        break;
    }

    // -----------------------------------------------------------------------
    case Phase::DONE:
        Serial.println();
        Serial.println(F("================================================"));
        Serial.print(F("  Results: "));
        Serial.print(_passed); Serial.print(F(" passed, "));
        Serial.print(_failed); Serial.println(F(" failed"));
        if (_failed == 0) {
            Serial.println(F("  All tests PASSED — hardware is operational."));
        } else {
            Serial.println(F("  See FAIL items above for corrective action."));
        }
        Serial.println(F("================================================"));
        while (true) { delay(1000); }   // halt — reflash to re-run
        break;
    }
}
