#include "telemetry.h"

// =============================================================================
// Constructor
// =============================================================================

Telemetry::Telemetry(MotionController& mc, StateMachine& sm)
    : _mc            (mc)
    , _sm            (sm)
    , _rateHz        (DEFAULT_TELEM_RATE_HZ)
    , _intervalMs    (DEFAULT_TELEM_RATE_HZ > 0 ? 1000u / DEFAULT_TELEM_RATE_HZ : 0u)
    , _lastBroadcastMs(0)
{}

// =============================================================================
// Public interface
// =============================================================================

void Telemetry::begin() {
    // Seed the timestamp so the first broadcast fires one full interval after
    // startup rather than immediately.  This avoids a burst of output before
    // the Python UI has had time to open the serial port.
    _lastBroadcastMs = millis();
}

void Telemetry::update() {
    // Rate of 0 means streaming is disabled — nothing to do.
    if (_intervalMs == 0) return;

    uint32_t now = millis();

    // Unsigned subtraction handles millis() rollover correctly (every ~49 days).
    if ((now - _lastBroadcastMs) >= _intervalMs) {
        // Anchor the next interval to the scheduled time, not to now.
        // This prevents slow drift caused by loop() jitter accumulating over
        // many broadcasts.
        _lastBroadcastMs += _intervalMs;

        // If we have fallen more than one interval behind (e.g. a long ISR or
        // heavy serial burst), resync to now so we do not fire a burst of
        // catch-up broadcasts.
        if ((now - _lastBroadcastMs) >= _intervalMs) {
            _lastBroadcastMs = now;
        }

        broadcast();
    }
}

void Telemetry::setRate(uint8_t hz) {
    if (hz > MAX_TELEM_RATE_HZ) hz = MAX_TELEM_RATE_HZ;
    _rateHz     = hz;
    // Integer division gives the nearest whole-millisecond interval.
    // At 50 Hz this is 20 ms; at 1 Hz it is 1000 ms.  A rate of 0
    // sets _intervalMs to 0 which disables broadcasting in update().
    _intervalMs = (hz > 0) ? (1000u / hz) : 0u;
}

uint8_t Telemetry::rate() const {
    return _rateHz;
}

// =============================================================================
// Private helpers
// =============================================================================

void Telemetry::broadcast() {
    // Fields are printed with individual Serial.print() calls rather than
    // snprintf() + Serial.print() to avoid a 128-byte stack buffer and the
    // associated risk of truncation on the STM32's limited stack.
    //
    // Field order must match the Python TelemetryFrame dataclass exactly:
    //   TELEM,<millis_ms>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>
    //
    // Velocities: actual comes from the encoder (getRPM), commanded is the
    // last value written to the stepper.  Both are broadcast so the Python UI
    // can compare them and verify profile fidelity.
    Serial.print("TELEM,");
    Serial.print(millis());
    Serial.print(',');
    Serial.print(_mc.getPositionMm(),           2);  // mm, 2 dp
    Serial.print(',');
    Serial.print(_mc.getActualVelocityMms(),    2);  // mm/s encoder
    Serial.print(',');
    Serial.print(_mc.getCommandedVelocityMms(), 2);  // mm/s commanded
    Serial.print(',');
    Serial.print(_mc.getActualAccelMms2(),      2);  // mm/s² (0 — not yet impl.)
    Serial.print(',');
    Serial.print(_sm.stateString());                 // e.g. "RUNNING"
    Serial.print(',');
    Serial.println(_sm.phaseString());               // e.g. "DESCENDING"
}
