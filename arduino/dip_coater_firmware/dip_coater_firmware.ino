// =============================================================================
// dip_coater_firmware.ino — Main sketch
//
// Board  : uStepper S32  (Tools > Board > uStepper STM32 Boards)
// Library: uStepperS32   (Library Manager)
// Baud   : 115200
//
// Module overview
// ---------------
//   StateMachine     — tracks system state (IDLE / HOMING / READY / RUNNING /
//                      PAUSED / ERROR) and the active run phase
//   MotionController — owns the stepper driver, executes all motion commands
//   Diagnostics      — hardware self-tests triggered by serial DIAG commands
//   CommandParser    — owns all serial I/O, dispatches CMD and DIAG commands
//   Telemetry        — streams position/velocity to Python UI  (step 7 — TODO)
//
// Coordinate system
// -----------------
//   Home (top endstop) = 0 mm
//   Positive mm        = upward   (toward home)
//   Negative mm        = downward (into solution)
// =============================================================================

#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"
#include "command_parser.h"

// -----------------------------------------------------------------------------
// Global objects
// -----------------------------------------------------------------------------

StateMachine     sm;
MotionController mc(sm);
Diagnostics      diag;
CommandParser    commandParser(sm, mc, diag);

// -----------------------------------------------------------------------------
// Endstop ISR wrappers
//
// While ENDSTOP_TEST mode is active, manual endstop triggers are absorbed here
// so they do not activate the limit-backoff logic in MotionController.
// -----------------------------------------------------------------------------

void onBottomEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(false);
}

void onTopEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(true);
}

// =============================================================================
// setup()
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) { delay(10); }

    pinMode(PIN_ENDSTOP_BOTTOM, INPUT_PULLUP);
    pinMode(PIN_ENDSTOP_TOP,    INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_BOTTOM), onBottomEndstop, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP_TOP),    onTopEndstop,    FALLING);

    mc.begin();
    diag.begin(mc, sm);
    commandParser.begin();

    Serial.println("Dip coater ready. State: IDLE");
    Serial.println("Type HELP for command list.");
}

// =============================================================================
// loop()
// =============================================================================

void loop() {
    commandParser.update();   // read serial, dispatch commands
    mc.update();              // service active motion mode
    diag.update();            // service active diagnostic test
    // TODO step 7: telemetry.update();
}
