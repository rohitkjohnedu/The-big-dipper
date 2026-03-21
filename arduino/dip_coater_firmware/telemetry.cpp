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
    _lastBroadcastMs = millis();
}

void Telemetry::update() {
    // Rate of 0 means disabled — nothing to do.
    if (_intervalMs == 0) return;

    uint32_t now = millis();
    if ((now - _lastBroadcastMs) >= _intervalMs) {
        _lastBroadcastMs = now;
        broadcast();
    }
}

void Telemetry::setRate(uint8_t hz) {
    if (hz > MAX_TELEM_RATE_HZ) hz = MAX_TELEM_RATE_HZ;
    _rateHz      = hz;
    _intervalMs  = (hz > 0) ? (1000u / hz) : 0u;
}

uint8_t Telemetry::rate() const {
    return _rateHz;
}

// =============================================================================
// Private helpers
// =============================================================================

void Telemetry::broadcast() {
    // TELEM,<millis>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>
    Serial.print("TELEM,");
    Serial.print(millis());
    Serial.print(',');
    Serial.print(_mc.getPositionMm(),          2);
    Serial.print(',');
    Serial.print(_mc.getActualVelocityMms(),   2);
    Serial.print(',');
    Serial.print(_mc.getCommandedVelocityMms(),2);
    Serial.print(',');
    Serial.print(_mc.getActualAccelMms2(),     2);
    Serial.print(',');
    Serial.print(_sm.stateString());
    Serial.print(',');
    Serial.println(_sm.phaseString());
}
