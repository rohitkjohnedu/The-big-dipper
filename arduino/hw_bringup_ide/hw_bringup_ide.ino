// =============================================================================
// hw_bringup_ide.ino — Hardware bringup test (Arduino IDE version)
//
// Board  : uStepper STM32 Boards (select in Tools > Board)
// Library: uStepperS32 (installed via Library Manager)
//
// Open Serial Monitor at 115200 baud BEFORE pressing Reset.
//
// Test sequence:
//   TEST 1  Motor connectivity  — jog DOWN 2 s, encoder must move >= 3 mm
//   TEST 2  Homing              — drive UP to top endstop, zero encoder, back off
//   TEST 3  Move accuracy (down)— jog DOWN 10 s at 5 mm/s, compare encoder vs commanded
//   TEST 4  Move accuracy (up)  — jog UP  5 s at 5 mm/s, compare encoder vs commanded
// =============================================================================

#include <UstepperS32.h>

// ---- Pin assignments (must match config.h) ----------------------------------
#define PIN_TOP_ENDSTOP    3   // D3, active-LOW
#define PIN_BOTTOM_ENDSTOP 2   // D2, active-LOW

// ---- Machine constants ------------------------------------------------------
#define LEADSCREW_MM_PER_REV  8.0f
#define STEPS_PER_MM         800.0f   // 200 steps * 32 microsteps / 8 mm

// ---- Test parameters --------------------------------------------------------
#define JOG_SPEED_MMS        5.0f
#define PRESENCE_JOG_MS      2000     //  2 s →  10 mm
#define PRESENCE_MIN_MM      3.0f
#define HOME_SPEED_MMS       3.0f
#define HOME_TIMEOUT_MS      30000
#define BACKOFF_MM           5.0f
#define ACCURACY_DOWN_MS     10000    // 10 s →  50 mm
#define ACCURACY_UP_MS       5000     //  5 s →  25 mm
#define POSITION_TOL_MM      2.0f
#define SETTLE_MS            600

// ---- Globals ----------------------------------------------------------------
UstepperS32 stepper;

volatile bool topTriggered = false;
volatile bool botTriggered = false;

int passed = 0, failed = 0;

// ---- ISR callbacks ----------------------------------------------------------
void onTopEndstop() {
  topTriggered = true;
  stepper.stop(0);   // HARD stop
}

void onBotEndstop() {
  botTriggered = true;
  stepper.stop(0);
}

// ---- Helpers ----------------------------------------------------------------
float positionMm() {
  return stepper.angleMoved() * (LEADSCREW_MM_PER_REV / 360.0f);
}

void setSpeed(float mms) {
  float sps = mms * STEPS_PER_MM;
  stepper.setMaxVelocity(sps);
  stepper.setMaxAcceleration(sps * 2.0f);
  stepper.setMaxDeceleration(sps * 2.0f);
}

void printCheck(const char* name, bool ok) {
  Serial.print(ok ? "  [PASS] " : "  [FAIL] ");
  Serial.println(name);
  if (ok) passed++; else failed++;
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }   // wait for Serial Monitor to open

  Serial.println("================================================");
  Serial.println("  Dip Coater - Hardware Bringup Test");
  Serial.println("================================================");
  Serial.println();

  // Endstop pins
  pinMode(PIN_TOP_ENDSTOP,    INPUT_PULLUP);
  pinMode(PIN_BOTTOM_ENDSTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_TOP_ENDSTOP),    onTopEndstop, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTTOM_ENDSTOP), onBotEndstop, FALLING);

  // Motor init: NORMAL mode, 200 steps/rev, default PID, no auto-home
  stepper.setup(NORMAL, 200, 10.0f, 0.0f, 0.0f, 16, false, 0, 50, 30);
  Serial.println("Motor initialised.");
  Serial.println();

  // ===========================================================================
  // TEST 1: Motor Connectivity
  // ===========================================================================
  Serial.println("--- TEST 1: Motor Connectivity ---");
  Serial.print("  Jogging DOWN at "); Serial.print(JOG_SPEED_MMS, 1);
  Serial.print(" mm/s for "); Serial.print(PRESENCE_JOG_MS / 1000);
  Serial.println(" s.");
  Serial.println("  (positive displacement = down is correct)");

  setSpeed(JOG_SPEED_MMS);
  float t1start = positionMm();
  botTriggered = false;
  stepper.runContinous(CW);      // CW = down — swap to CCW if direction is wrong
  delay(PRESENCE_JOG_MS);
  stepper.stop(1);               // SOFT stop
  delay(SETTLE_MS);

  float t1dist = positionMm() - t1start;
  Serial.print("  Displacement: "); Serial.print(t1dist, 2); Serial.println(" mm");
  if (t1dist < 0) {
    Serial.println("  NOTE: negative displacement means CW = UP on your wiring.");
    Serial.println("        Swap CW <-> CCW in this sketch and in motion_controller.cpp");
  }
  printCheck("Motor moves >= 3 mm in 2 s", fabsf(t1dist) >= PRESENCE_MIN_MM);

  // ===========================================================================
  // TEST 2: Homing
  // ===========================================================================
  Serial.println();
  Serial.println("--- TEST 2: Homing ---");
  Serial.println("  Driving UP (CCW) toward top endstop...");

  topTriggered = false;
  setSpeed(HOME_SPEED_MMS);
  stepper.runContinous(CCW);     // CCW = up

  uint32_t homeStart = millis();
  while (!topTriggered && (millis() - homeStart < HOME_TIMEOUT_MS)) {
    delay(10);
  }

  if (!topTriggered) {
    Serial.println("  TIMEOUT: top endstop not triggered in 30 s. Stopping.");
    stepper.stop(0);
    printCheck("Top endstop triggered", false);
    goto summary;
  }

  printCheck("Top endstop triggered", true);
  stepper.encoder.setHome();    // zero position at top endstop

  // Back off downward
  Serial.print("  Backing off "); Serial.print(BACKOFF_MM, 1); Serial.println(" mm down.");
  setSpeed(HOME_SPEED_MMS);
  stepper.moveToAngle(BACKOFF_MM * 360.0f / LEADSCREW_MM_PER_REV);
  delay(4000);   // wait for move to complete (5 mm at 3 mm/s = 1.7 s + margin)

  {
    float homePos = positionMm();
    Serial.print("  Position after home: "); Serial.print(homePos, 2);
    Serial.print(" mm  (expected "); Serial.print(BACKOFF_MM, 1); Serial.println(" mm)");
    printCheck("Post-home position within 2 mm of backoff", fabsf(homePos - BACKOFF_MM) <= POSITION_TOL_MM);
  }

  // ===========================================================================
  // TEST 3: Move Accuracy (Down)
  // ===========================================================================
  {
    Serial.println();
    Serial.println("--- TEST 3: Move Accuracy (Down) ---");
    float expected = JOG_SPEED_MMS * (ACCURACY_DOWN_MS / 1000.0f);
    Serial.print("  Jogging DOWN "); Serial.print(expected, 1);
    Serial.print(" mm  ("); Serial.print(JOG_SPEED_MMS, 1);
    Serial.print(" mm/s x "); Serial.print(ACCURACY_DOWN_MS / 1000); Serial.println(" s)");

    setSpeed(JOG_SPEED_MMS);
    float t3start = positionMm();
    botTriggered = false;
    stepper.runContinous(CW);
    delay(ACCURACY_DOWN_MS);
    stepper.stop(1);
    delay(SETTLE_MS);

    if (botTriggered) Serial.println("  WARN: bottom endstop hit — reduce depth or start position");

    float actual = positionMm() - t3start;
    float err    = fabsf(actual - expected);
    Serial.print("  Expected: "); Serial.print(expected, 2); Serial.println(" mm");
    Serial.print("  Actual  : "); Serial.print(actual,   2); Serial.println(" mm");
    Serial.print("  Error   : "); Serial.print(err,      2); Serial.println(" mm");
    printCheck("Down displacement within 2 mm of commanded", err <= POSITION_TOL_MM);
  }

  // ===========================================================================
  // TEST 4: Move Accuracy (Up)
  // ===========================================================================
  {
    Serial.println();
    Serial.println("--- TEST 4: Move Accuracy (Up) ---");
    float expected = JOG_SPEED_MMS * (ACCURACY_UP_MS / 1000.0f);
    Serial.print("  Jogging UP "); Serial.print(expected, 1);
    Serial.print(" mm  ("); Serial.print(JOG_SPEED_MMS, 1);
    Serial.print(" mm/s x "); Serial.print(ACCURACY_UP_MS / 1000); Serial.println(" s)");

    setSpeed(JOG_SPEED_MMS);
    float t4start = positionMm();
    stepper.runContinous(CCW);
    delay(ACCURACY_UP_MS);
    stepper.stop(1);
    delay(SETTLE_MS);

    float actual   = positionMm() - t4start;   // negative = upward
    float err      = fabsf(fabsf(actual) - expected);
    Serial.print("  Expected: -"); Serial.print(expected, 2); Serial.println(" mm");
    Serial.print("  Actual  : ");  Serial.print(actual,   2); Serial.println(" mm");
    Serial.print("  Error   : ");  Serial.print(err,      2); Serial.println(" mm");
    printCheck("Up displacement within 2 mm of commanded", err <= POSITION_TOL_MM);
  }

summary:
  Serial.println();
  Serial.println("================================================");
  Serial.print("  Results: "); Serial.print(passed);
  Serial.print(" passed, ");   Serial.print(failed); Serial.println(" failed");
  if (failed == 0)
    Serial.println("  All tests PASSED - hardware is operational.");
  else
    Serial.println("  See FAIL lines above for corrective action.");
  Serial.println("================================================");
}

void loop() {
  // All tests run once in setup().
}
