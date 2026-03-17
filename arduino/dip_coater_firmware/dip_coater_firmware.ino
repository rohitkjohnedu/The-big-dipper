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
//   Diagnostics     — hardware self-test triggered by serial command
//   CommandParser   — owns all serial I/O, dispatches CMD and DIAG commands
//   Telemetry       — streams position/velocity to Python UI (step 7 — TODO)
// =============================================================================

#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"
#include "command_parser.h"

// --- Global objects ----------------------------------------------------------
StateMachine     sm;
MotionController mc(sm);
Diagnostics      diag;
CommandParser    commandParser(sm, mc, diag);

// --- Endstop ISR wrappers ----------------------------------------------------
// In ENDSTOP_TEST mode the ISR is bypassed so manual triggers don't trip the
// limit backoff logic.
void onBottomEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(false);
}
void onTopEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(true);
}

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

void loop() {
    commandParser.update();
    mc.update();
    diag.update();
    // TODO step 7: telemetry.update() here
}
