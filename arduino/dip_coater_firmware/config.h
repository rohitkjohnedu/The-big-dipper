#pragma once

// Motor / drive
#define MOTOR_STEPS_PER_REV       200
#define MICROSTEPS                32
#define LEADSCREW_MM_PER_REV      8.0f
#define STEPS_PER_MM              800.0f   // (200 * 32) / 8.0

// Travel limits
#define TRAVEL_MAX_MM             900.0f
#define SOFT_LIMIT_MIN_MM         10.0f
#define SOFT_LIMIT_MAX_MM         890.0f

// Homing
#define HOMING_SPEED_MM_S         20.0f
#define HOMING_BACKOFF_MM         5.0f
#define HOMING_ACC_MM_S2          20.0f

// Limit switch backoff — how far to move away after an endstop is hit
#define LIMIT_BACKOFF_MM          5.0f
#define LIMIT_BACKOFF_SPEED_MM_S  10.0f

// Pins
#define PIN_ENDSTOP_BOTTOM        2
#define PIN_ENDSTOP_TOP           3
#define PIN_STATUS_LED            13

// Serial
#define SERIAL_BAUD_RATE          115200
#define SERIAL_BUFFER_SIZE        128
#define DEFAULT_TELEM_RATE_HZ     1
#define MAX_TELEM_RATE_HZ         50

// Default profile parameters
#define DEFAULT_DIP_SPEED_MM_S        10.0f
#define DEFAULT_WITHDRAW_SPEED_MM_S   10.0f
#define DEFAULT_ACCEL_MM_S2           50.0f
#define DEFAULT_DIP_DEPTH_MM          100.0f
#define DEFAULT_DWELL_BOTTOM_MS       1000
#define DEFAULT_DWELL_TOP_MS          500
#define DEFAULT_N_DIPS                1

#define DEBUG_DEFAULT_ON          0

// Segmented move buffer
#define MOVE_SEG_BUFFER_SIZE      64
