// =============================================================================
// endstop_test.cpp — Standalone endstop diagnostic
//
// Flash:   pio run -e endstop_test -t upload
// Monitor: pio device monitor -e endstop_test   (115200 baud)
//
// Manually press and release each limit switch.
// Serial output uses a structured format so a future diagnostics UI can parse it:
//
//   ENDSTOP:BOTTOM:TRIGGERED     <- switch closed (pin LOW)
//   ENDSTOP:BOTTOM:RELEASED      <- switch opened (pin HIGH)
//   ENDSTOP:TOP:TRIGGERED
//   ENDSTOP:TOP:RELEASED
//   ENDSTOP:STATUS:BOTTOM=RELEASED,TOP=RELEASED   <- on startup and on 's' key
//
// Send 's' over serial at any time to query the current state of both switches.
// =============================================================================

#include <Arduino.h>

// Pin assignments — must match config.h PIN_ENDSTOP_BOTTOM / PIN_ENDSTOP_TOP
static constexpr uint8_t PIN_BOTTOM  = 2;
static constexpr uint8_t PIN_TOP     = 3;
static constexpr uint8_t DEBOUNCE_MS = 20;

// ---------------------------------------------------------------------------
// Endstop descriptor
// ---------------------------------------------------------------------------

struct Endstop {
    uint8_t     pin;
    const char* name;
    bool        stable;     // last confirmed stable state (HIGH = open = not triggered)
    bool        reading;    // latest raw reading
    uint32_t    changedAt;  // millis() when reading last changed
};

static Endstop endstops[2] = {
    { PIN_BOTTOM, "BOTTOM", HIGH, HIGH, 0 },
    { PIN_TOP,    "TOP",    HIGH, HIGH, 0 },
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool isTriggered(const Endstop& e) {
    return e.stable == LOW;   // active-LOW with INPUT_PULLUP
}

static void printStatus() {
    Serial.print(F("ENDSTOP:STATUS:BOTTOM="));
    Serial.print(isTriggered(endstops[0]) ? F("TRIGGERED") : F("RELEASED"));
    Serial.print(F(",TOP="));
    Serial.println(isTriggered(endstops[1]) ? F("TRIGGERED") : F("RELEASED"));
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(500);

    for (auto& e : endstops) {
        pinMode(e.pin, INPUT_PULLUP);
        e.stable  = digitalRead(e.pin);
        e.reading = e.stable;
    }

    Serial.println(F("================================================"));
    Serial.println(F("  Dip Coater - Endstop Diagnostic"));
    Serial.println(F("================================================"));
    Serial.println(F("Manually trigger each switch."));
    Serial.println(F("Send 's' to query current state at any time."));
    Serial.println();
    printStatus();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------

void loop() {
    uint32_t now = millis();

    // --- Debounced edge detection -----------------------------------------
    for (auto& e : endstops) {
        bool current = digitalRead(e.pin);

        if (current != e.reading) {
            // New reading: restart the debounce window
            e.reading   = current;
            e.changedAt = now;
        } else if (current != e.stable && (now - e.changedAt) >= DEBOUNCE_MS) {
            // Stable for DEBOUNCE_MS: commit and report
            e.stable = current;
            Serial.print(F("ENDSTOP:"));
            Serial.print(e.name);
            Serial.print(F(":"));
            Serial.println(isTriggered(e) ? F("TRIGGERED") : F("RELEASED"));
        }
    }

    // --- Serial command handler ('s' = status query) ----------------------
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            printStatus();
        }
    }
}
