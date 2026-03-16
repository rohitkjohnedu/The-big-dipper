// =============================================================================
// dip_coater_firmware.ino  — Main sketch
//
// Board  : uStepper S32 (Tools > Board > uStepper STM32 Boards)
// Library: uStepperS32 (Library Manager)
// Baud   : 115200
//
// Architecture:
//   StateMachine    — tracks system state (IDLE/HOMING/READY/RUNNING/PAUSED/ERROR)
//   MotionController— owns the stepper, executes all motion
//   CommandParser   — parses serial commands from Python UI  (step 6 — TODO)
//   Telemetry       — streams position/velocity to Python UI (step 7 — TODO)
// =============================================================================

#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"

// --- Global objects ----------------------------------------------------------
StateMachine     sm;
MotionController mc(sm);

// --- Endstop ISR wrappers ----------------------------------------------------
void onBottomEndstop() { mc.onEndstopTriggered(false); }
void onTopEndstop()    { mc.onEndstopTriggered(true);  }

// =============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) { delay(10); }

  pinMode(PIN_ENDSTOP_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_ENDSTOP_TOP,    INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_BOTTOM), onBottomEndstop, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_TOP),    onTopEndstop,    FALLING);

  mc.begin();

  Serial.println("Dip coater ready. State: IDLE");
  // TODO step 6: initialise CommandParser and Telemetry here
}

void loop() {
  mc.update();
  // TODO step 6: commandParser.update() and telemetry.update() here
}
