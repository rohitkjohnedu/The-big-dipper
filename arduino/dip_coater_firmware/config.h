#pragma once

/**
 * @file   config.h
 * @brief  Compile-time constants for the dip coater firmware.
 *
 * All physical parameters, pin assignments, default profile values, and
 * driver tuning registers live here.  Change these to match your hardware
 * without touching any logic files.
 *
 * Coordinate system
 * -----------------
 *   Home (top endstop) = 0 mm
 *   Positive mm        = upward   (toward home)
 *   Negative mm        = downward (into solution)
 */

// =============================================================================
// Motor / drive train
// =============================================================================

#define MOTOR_STEPS_PER_REV      200        ///< Full steps per motor revolution
#define MICROSTEPS               32         ///< Microstepping divisor
#define LEADSCREW_MM_PER_REV     8.0f       ///< Linear travel per motor revolution (mm)
#define STEPS_PER_MM             800.0f     ///< Derived: (200 × 32) / 8.0

// =============================================================================
// Travel limits
// =============================================================================

#define TRAVEL_MAX_MM            900.0f     ///< Maximum upward command used during homing
#define SOFT_LIMIT_MIN_MM       -890.0f     ///< Maximum downward travel (negative = down)
#define SOFT_LIMIT_MAX_MM         10.0f     ///< Small positive margin above home

// =============================================================================
// Homing
// =============================================================================

#define HOMING_SPEED_MM_S        20.0f      ///< Speed for the approach to the top endstop (mm/s)
#define HOMING_BACKOFF_MM         5.0f      ///< Distance to back off from the endstop after contact (mm)
#define HOMING_ACC_MM_S2         20.0f      ///< Acceleration used during homing (mm/s²)

// =============================================================================
// Limit-switch backoff
// =============================================================================

#define LIMIT_BACKOFF_MM          5.0f      ///< Distance to back away from an endstop after an unexpected trigger (mm)
#define LIMIT_BACKOFF_SPEED_MM_S 10.0f      ///< Speed used during the backoff move (mm/s)

// =============================================================================
// Pin assignments
// =============================================================================

#define PIN_ENDSTOP_BOTTOM        2         ///< Bottom endstop — digital input with pull-up, active LOW
#define PIN_ENDSTOP_TOP           3         ///< Top endstop  — digital input with pull-up, active LOW
#define PIN_STATUS_LED           13         ///< Onboard status LED

// =============================================================================
// Serial / telemetry
// =============================================================================

#define SERIAL_BAUD_RATE         115200     ///< UART baud rate
#define SERIAL_BUFFER_SIZE       128        ///< Maximum incoming command length (bytes)
#define DEFAULT_TELEM_RATE_HZ    1          ///< Default telemetry streaming rate (Hz)
#define MAX_TELEM_RATE_HZ        50         ///< Maximum telemetry streaming rate (Hz)

// =============================================================================
// Default dip-profile parameters
// =============================================================================

#define DEFAULT_DIP_SPEED_MM_S       10.0f  ///< Default descent speed (mm/s)
#define DEFAULT_WITHDRAW_SPEED_MM_S  10.0f  ///< Default withdrawal speed (mm/s)
#define DEFAULT_ACCEL_MM_S2          50.0f  ///< Default acceleration / deceleration (mm/s²)
#define DEFAULT_DIP_DEPTH_MM        100.0f  ///< Default dip depth below home (mm)
#define DEFAULT_DWELL_BOTTOM_MS      1000   ///< Default time to dwell in solution (ms)
#define DEFAULT_DWELL_TOP_MS          500   ///< Default time to dwell at top between dips (ms)
#define DEFAULT_N_DIPS                  1   ///< Default number of dip cycles

// =============================================================================
// StealthChop tuning  (TMC5130 registers written in MotionController::begin())
// =============================================================================
//
// TPWMTHRS — crossover speed between StealthChop (quiet) and SpreadCycle (torque).
//   StealthChop is active when the motor is SLOWER than this threshold.
//
//   Conversion: TPWMTHRS = 1562 / crossover_speed_mm_s
//     1 mm/s → 1562 |  2 mm/s → 781 |  3 mm/s → 521 |  5 mm/s → 312
//
//   Set to 0 to force StealthChop at all speeds (motor may stall above ~2 mm/s).
//   Tune upward if slow moves are noisy; downward if fast moves lose torque.
//
// TPOWERDOWN — delay before hold current activates after standstill (~2 ms per count).
//   Prevents the audible click when the motor stops.  10 ≈ 20 ms.

#define STEALTH_TPWMTHRS         312        ///< Crossover at ~5 mm/s
#define STEALTH_TPOWERDOWN        10        ///< ~20 ms hold-current delay

// =============================================================================
// Segmented move buffer
// =============================================================================

#define MOVE_SEG_BUFFER_SIZE      64        ///< Maximum number of segments per loaded move

// =============================================================================
// Debug trace
// =============================================================================
//
// Uncomment to emit key motion-controller events to Serial.
// Format: "TR <millis_ms> <message>"
// Leave commented out during normal operation — output is noisy.

// #define TRACE
