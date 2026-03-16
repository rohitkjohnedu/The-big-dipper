// =============================================================================
// endstop_test.ino  — Standalone limit switch diagnostic
//
// Board  : uStepper S32 (Tools > Board)
// Baud   : 115200
// No motor connection required.
//
// Open Serial Monitor, then press Reset.
// Manually trigger each switch and observe the output.
// Output format is parseable by the Python diagnostics UI:
//   ENDSTOP:<name>:<TRIGGERED|RELEASED>
// =============================================================================

#define PIN_TOP_ENDSTOP    3   // D3 — top (home) endstop, active-LOW
#define PIN_BOTTOM_ENDSTOP 2   // D2 — bottom endstop,    active-LOW
#define DEBOUNCE_MS        20

bool lastTop = HIGH;
bool lastBot = HIGH;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(PIN_TOP_ENDSTOP,    INPUT_PULLUP);
  pinMode(PIN_BOTTOM_ENDSTOP, INPUT_PULLUP);

  Serial.println("=== Endstop Test ===");
  Serial.println("Trigger each switch manually.");
  Serial.println("Format: ENDSTOP:<name>:<TRIGGERED|RELEASED>");
  Serial.println();

  // Print initial state
  lastTop = digitalRead(PIN_TOP_ENDSTOP);
  lastBot = digitalRead(PIN_BOTTOM_ENDSTOP);
  Serial.print("TOP    (D3): "); Serial.println(lastTop == LOW ? "TRIGGERED" : "released");
  Serial.print("BOTTOM (D2): "); Serial.println(lastBot == LOW ? "TRIGGERED" : "released");
  Serial.println();
  Serial.println("Waiting for changes...");
}

void loop() {
  // Poll top endstop
  bool top = digitalRead(PIN_TOP_ENDSTOP);
  if (top != lastTop) {
    delay(DEBOUNCE_MS);
    top = digitalRead(PIN_TOP_ENDSTOP);
    if (top != lastTop) {
      lastTop = top;
      Serial.print("ENDSTOP:TOP:");
      Serial.println(top == LOW ? "TRIGGERED" : "RELEASED");
    }
  }

  // Poll bottom endstop
  bool bot = digitalRead(PIN_BOTTOM_ENDSTOP);
  if (bot != lastBot) {
    delay(DEBOUNCE_MS);
    bot = digitalRead(PIN_BOTTOM_ENDSTOP);
    if (bot != lastBot) {
      lastBot = bot;
      Serial.print("ENDSTOP:BOTTOM:");
      Serial.println(bot == LOW ? "TRIGGERED" : "RELEASED");
    }
  }
}
