#ifndef ESC_H
#define ESC_H

#include <Arduino.h>
#include "lib/DShotRMT/DShotRMT.h"

// ESC DShot configuration
#define ESC_PIN 19
#define ESC_RMT_CHANNEL 0
#define ESC_STARTUP_MILLISECONDS 2000  // 2 seconds of 0 commands for arming

// DShot throttle mapping: 0-100% throttle to DShot values
// DShot range: 48 (min throttle) to 1047 (max throttle)
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 1047

// Global ESC object (declared in esc.cpp)
extern DShotRMT escMotor;

// Function declarations
void setESCDShot(uint16_t throttlePercent);  // 0-100% throttle

#endif // ESC_H

