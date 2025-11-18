#ifndef ESC_H
#define ESC_H

#include <Arduino.h>

// ============================================================================
// ESC PROTOCOL SELECTION - Change this to switch between DShot and PWM
// ============================================================================
#define USE_DSHOT 1  // Set to 1 for DShot600, 0 for PWM
// ============================================================================

#define ESC_PIN 19
#define ESC_RMT_CHANNEL 0
#define ESC_STARTUP_MILLISECONDS 2000  // 2 seconds for DShot arming

#if USE_DSHOT
  #include "lib/DShotRMT/DShotRMT.h"
  
  // DShot throttle mapping: 0-100% throttle to DShot values
  // DShot range: 48 (min throttle) to 1047 (max throttle)
  #define DSHOT_MIN_THROTTLE 48
  #define DSHOT_MAX_THROTTLE 1047
  
  // Global ESC object (declared in esc.cpp)
  extern DShotRMT escMotor;
#else
  // PWM configuration (was working for basic spin)
  #define PWM_FREQUENCY 50  // Hz
  #define PWM_RESOLUTION 16  // bits
  #define MIN_PULSE_WIDTH 1006  // microseconds (keeps ESC armed)
  #define MAX_PULSE_WIDTH 2006  // microseconds
  #define PULSE_RANGE (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#endif

// Unified function declaration - works for both DShot and PWM
void setESCThrottle(uint16_t throttlePercent);  // 0-100% throttle

// ESC initialization
void initESC();

#endif // ESC_H

