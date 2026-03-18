#pragma once
// =============================================================================
// Trace macros — enable with #define TRACE in config.h
// Output format: "TR <millis> <message>"
// =============================================================================

#ifdef TRACE
  #define TR(msg) do { \
      Serial.print(F("TR ")); Serial.print(millis()); \
      Serial.print(' '); Serial.println(F(msg)); } while(0)
  #define TRF(msg, val) do { \
      Serial.print(F("TR ")); Serial.print(millis()); \
      Serial.print(' '); Serial.print(F(msg)); Serial.println(val, 2); } while(0)
  #define TR2F(msg, v1, sep, v2) do { \
      Serial.print(F("TR ")); Serial.print(millis()); \
      Serial.print(' '); Serial.print(F(msg)); Serial.print(v1, 2); \
      Serial.print(F(sep)); Serial.println(v2, 2); } while(0)
#else
  #define TR(msg)
  #define TRF(msg, val)
  #define TR2F(msg, v1, sep, v2)
#endif
