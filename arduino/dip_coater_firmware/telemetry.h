#pragma once
#include <Arduino.h>
#include "config.h"
#include "motion_controller.h"
#include "state_machine.h"

/**
 * @file  telemetry.h
 * @brief Rate-limited telemetry broadcaster for the dip coater.
 *
 * Telemetry is streamed to the Python UI as CSV lines at a configurable rate
 * (default DEFAULT_TELEM_RATE_HZ, max MAX_TELEM_RATE_HZ).  A rate of 0 disables
 * streaming entirely.
 *
 * Output format (one line per interval):
 *   TELEM,<millis_ms>,<pos_mm>,<vel_actual_mm_s>,<vel_commanded_mm_s>,<accel_mm_s2>,<state>,<phase>
 *
 * Example:
 *   TELEM,12453,45.32,10.00,10.00,0.00,RUNNING,DESCENDING
 *
 * Usage
 * -----
 *   Call begin() once from setup().
 *   Call update() every loop() — it self-throttles to the configured rate.
 *   Call setRate() when CMD SET_TELEM_RATE is received.
 */
class Telemetry {
public:

    /**
     * @brief Construct with references to the motion controller and state machine.
     * @param mc  MotionController to read position/velocity from.
     * @param sm  StateMachine to read state/phase strings from.
     */
    Telemetry(MotionController& mc, StateMachine& sm);

    /** @brief Initialise timing state.  Call once from setup(). */
    void begin();

    /** @brief Check interval and broadcast one TELEM line if due.  Call every loop(). */
    void update();

    /**
     * @brief Set the broadcast rate.
     * @param hz  Broadcasts per second.  0 = disabled.  Clamped to MAX_TELEM_RATE_HZ.
     */
    void setRate(uint8_t hz);

    /** @brief Returns the current broadcast rate (Hz). */
    uint8_t rate() const;

private:

    MotionController& _mc;
    StateMachine&     _sm;

    uint8_t  _rateHz;         ///< Current broadcast rate (Hz); 0 = disabled
    uint32_t _intervalMs;     ///< Milliseconds between broadcasts (0 when disabled)
    uint32_t _lastBroadcastMs;///< millis() value at the last broadcast

    /** @brief Format and print one TELEM CSV line to Serial. */
    void broadcast();
};
