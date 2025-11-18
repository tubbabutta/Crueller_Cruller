#include "esc.h"

// Global DShot ESC object
DShotRMT escMotor(ESC_PIN, (rmt_channel_t)ESC_RMT_CHANNEL);

/**
 * Set ESC throttle via DShot
 * @param throttlePercent 0-100% throttle
 */
void setESCDShot(uint16_t throttlePercent) {
  if (throttlePercent > 100) throttlePercent = 100;
  
  uint16_t dshotValue;
  if (throttlePercent == 0) {
    dshotValue = 0;  // 0 keeps ESC armed
  } else {
    // Map 1-100% to 48-1047 DShot range
    dshotValue = DSHOT_MIN_THROTTLE + ((uint32_t)throttlePercent * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)) / 100;
  }
  
  escMotor.sendThrottleValue(dshotValue);
}
