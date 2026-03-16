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
//   CommandParser   — parses serial commands from Python UI  (step 6 — TODO)
//   Telemetry       — streams position/velocity to Python UI (step 7 — TODO)
//
// Diagnostic serial commands:
//   DIAG MOTOR    — run motor connectivity + accuracy test sequence
//   DIAG ENDSTOP  — live endstop monitoring (trigger manually)
//   DIAG EXIT     — return to standby
// =============================================================================

#include "config.h"
#include "state_machine.h"
#include "motion_controller.h"
#include "diagnostics.h"

// --- Global objects ----------------------------------------------------------
StateMachine     sm;
MotionController mc(sm);
Diagnostics      diag;

// --- Endstop ISR wrappers ----------------------------------------------------
// In ENDSTOP_TEST mode the ISR is bypassed so manual triggers don't emergency-stop.
void onBottomEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(false);
}
void onTopEndstop() {
    if (diag.mode() == Diagnostics::Mode::ENDSTOP_TEST) return;
    mc.onEndstopTriggered(true);
}

// --- Serial command buffer ---------------------------------------------------
static char    _cmdBuf[64];
static uint8_t _cmdLen = 0;

// Trim trailing \r\n and dispatch
static void dispatchCommand(char* line) {
    // Strip trailing whitespace
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';

    if (strcmp(line, "DIAG MOTOR") == 0) {
        diag.startMotorTest();
    } else if (strcmp(line, "DIAG MOTORENCODER") == 0) {
        diag.startMotorEncoderTest();
    } else if (strcmp(line, "DIAG ENDSTOP") == 0) {
        diag.startEndstopTest();
    } else if (strcmp(line, "DIAG EXIT") == 0) {
        diag.exit();
    } else if (strncmp(line, "DIAG JOG DOWN", 13) == 0) {
        float spd = *(line + 13) ? atof(line + 13) : 5.0f;
        diag.startJog(false, spd);
    } else if (strncmp(line, "DIAG JOG UP", 11) == 0) {
        float spd = *(line + 11) ? atof(line + 11) : 5.0f;
        diag.startJog(true, spd);
    } else if (strcmp(line, "DIAG JOG STOP") == 0) {
        diag.exit();
    } else if (strcmp(line, "DIAG POS") == 0) {
        diag.printPosition();
    } else if (strncmp(line, "DIAG MOVE", 9) == 0) {
        float mm = *(line + 9) ? atof(line + 9) : 10.0f;
        diag.startMoveTest(mm);
    } else {
        // TODO step 6: forward to CommandParser
        Serial.print("ERR:unknown command: ");
        Serial.println(line);
    }
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

    Serial.println("Dip coater ready. State: IDLE");
    Serial.println("Commands: DIAG MOVE [mm] | DIAG MOTOR | DIAG ENDSTOP | DIAG JOG DOWN [spd] | DIAG JOG UP [spd] | DIAG JOG STOP | DIAG POS | DIAG EXIT");
}

void loop() {
    // Read serial one byte at a time; dispatch on newline
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            _cmdBuf[_cmdLen] = '\0';
            dispatchCommand(_cmdBuf);
            _cmdLen = 0;
        } else if (_cmdLen < sizeof(_cmdBuf) - 1) {
            _cmdBuf[_cmdLen++] = c;
        }
    }

    mc.update();
    diag.update();
    // TODO step 6: commandParser.update() and telemetry.update() here
}
