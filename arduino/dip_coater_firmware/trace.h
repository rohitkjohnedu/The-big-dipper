#pragma once

/**
 * @file  trace.h
 * @brief Lightweight serial trace macros for motion-controller debugging.
 *
 * Enable by adding `#define TRACE` in config.h.
 * All macros expand to nothing when TRACE is not defined, so there is
 * zero runtime cost in normal builds.
 *
 * Output format:  TR <millis_ms> <message>
 *
 * Macros
 * ------
 *   TR(msg)              — print a plain string label
 *   TRF(msg, val)        — print a label followed by one float (2 dp)
 *   TR2F(msg,v1,sep,v2)  — print a label, two floats separated by sep
 */

#ifdef TRACE

    #define TR(msg) \
        do { \
            Serial.print(F("TR ")); \
            Serial.print(millis()); \
            Serial.print(' '); \
            Serial.println(F(msg)); \
        } while (0)

    #define TRF(msg, val) \
        do { \
            Serial.print(F("TR ")); \
            Serial.print(millis()); \
            Serial.print(' '); \
            Serial.print(F(msg)); \
            Serial.println(val, 2); \
        } while (0)

    #define TR2F(msg, v1, sep, v2) \
        do { \
            Serial.print(F("TR ")); \
            Serial.print(millis()); \
            Serial.print(' '); \
            Serial.print(F(msg)); \
            Serial.print(v1, 2); \
            Serial.print(F(sep)); \
            Serial.println(v2, 2); \
        } while (0)

#else

    #define TR(msg)
    #define TRF(msg, val)
    #define TR2F(msg, v1, sep, v2)

#endif
