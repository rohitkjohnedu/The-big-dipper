// =============================================================================
// dip_coater_firmware.ino — Main sketch
//
// Board  : uStepper S32  (Tools > Board > uStepper STM32 Boards)
// Library: uStepperS32   (Library Manager)
//          Pushbutton    (Pololu — add via Sketch > Include Library > Add .ZIP)
// Baud   : 115200
//
// Module overview
// ---------------
//   StateMachine     — tracks system state (IDLE / HOMING / READY / RUNNING /
//                      PAUSED / ERROR) and the active run phase
//   MotionController — owns the stepper driver, executes all motion commands
//   Diagnostics      — hardware self-tests triggered by serial DIAG commands
//   CommandParser    — owns all serial I/O, dispatches CMD and DIAG commands
//   Telemetry        — streams position/velocity/state to Python UI at a
//                      configurable rate (default DEFAULT_TELEM_RATE_HZ)
//
// Coordinate system
// -----------------
//   Home (top endstop) = 0 mm
//   Positive mm        = upward   (toward home)
//   Negative mm        = downward (into solution)
//
// Endstop debouncing
// ------------------
//   Both endstops are polled in loop() using the Pololu Pushbutton library.
//   getSingleDebouncedPress() runs a 15 ms state-machine debounce — it returns
//   true exactly once per confirmed trigger, filtering EMI and crosstalk spikes
//   without any delayMicroseconds() hacks in ISRs.  Interrupts are not used.
// =============================================================================

#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"
#include "command_parser.h"
#include "telemetry.h"
#include <Pushbutton.h>

// -----------------------------------------------------------------------------
// Global objects
// -----------------------------------------------------------------------------

StateMachine     sm;
MotionController mc(sm);
Diagnostics      diag;
Telemetry        telem(mc, sm);
CommandParser    commandParser(sm, mc, diag);

// Endstop inputs — active LOW with internal pull-up.
//   PULL_UP_ENABLED  : Pushbutton calls pinMode(pin, INPUT_PULLUP) on first use.
//   DEFAULT_STATE_HIGH: released state reads HIGH; triggered state reads LOW.
//   getSingleDebouncedPress() returns true once per debounced LOW transition.
Pushbutton endstopBottom(PIN_ENDSTOP_BOTTOM, PULL_UP_ENABLED, DEFAULT_STATE_HIGH);
Pushbutton endstopTop   (PIN_ENDSTOP_TOP,    PULL_UP_ENABLED, DEFAULT_STATE_HIGH);

// =============================================================================
// setup()
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) { delay(10); }

    // No explicit pinMode or attachInterrupt needed for the endstops.
    // Pushbutton configures INPUT_PULLUP lazily on the first isPressed() call
    // (inside the loop() polling below), and the library's state-machine
    // debounce replaces the interrupt-based approach entirely.

    mc.begin();
    diag.begin(mc, sm);
    telem.begin();
    commandParser.begin();
    commandParser.setTelemetry(&telem);

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
    telem.update();           // broadcast telemetry at configured rate

    // -------------------------------------------------------------------------
    // Endstop polling — Pushbutton 15 ms debounce
    //
    // getSingleDebouncedPress() advances an internal state machine on every
    // call and returns true exactly once after a pin has been continuously LOW
    // for 15 ms.  This rejects brief EMI spikes and capacitive crosstalk between
    // adjacent signal lines without blocking loop().
    //
    // In ENDSTOP_TEST mode diag.update() already monitors both pins directly;
    // the triggers are absorbed here so they do not reach MotionController.
    // -------------------------------------------------------------------------
    if (endstopBottom.getSingleDebouncedPress()) {
        if (diag.mode() != Diagnostics::Mode::ENDSTOP_TEST)
            mc.onEndstopTriggered(false);
    }
    if (endstopTop.getSingleDebouncedPress()) {
        if (diag.mode() != Diagnostics::Mode::ENDSTOP_TEST)
            mc.onEndstopTriggered(true);
    }
}
