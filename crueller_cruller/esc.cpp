#include "esc.h"

#if USE_DSHOT
  // ========================================================================
  // DSHOT IMPLEMENTATION - EXPERIMENTAL
  // ========================================================================
  DShotRMT escMotor(ESC_PIN, (rmt_channel_t)ESC_RMT_CHANNEL);
  
  void initESC() {
    escMotor.begin(DSHOT600);
  }
  
  void setESCThrottle(uint16_t throttlePercent) {
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
#else
  // ========================================================================
  // PWM IMPLEMENTATION - WORKING (was working for basic spin)
  // ========================================================================
  #include <driver/ledc.h>
  
  void initESC() {
    // Configure LEDC for PWM
    ledc_timer_config_t timerConfig = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_16_BIT,
      .freq_hz = PWM_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timerConfig);
    
    // Configure LEDC channel
    ledc_channel_config_t channelConfig = {
      .gpio_num = ESC_PIN,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER_0,
      .intr_type = LEDC_INTR_DISABLE,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&channelConfig);
    
    // Start with minimum pulse to arm ESC
    setESCThrottle(0);
  }
  
  void setESCThrottle(uint16_t throttlePercent) {
    if (throttlePercent > 100) throttlePercent = 100;
    
    // Calculate pulse width in microseconds
    uint16_t pulseWidth_us;
    if (throttlePercent == 0) {
      pulseWidth_us = MIN_PULSE_WIDTH;  // Minimum pulse keeps ESC armed
    } else {
      // Map 1-100% to MIN_PULSE_WIDTH to MAX_PULSE_WIDTH
      pulseWidth_us = MIN_PULSE_WIDTH + ((uint32_t)throttlePercent * PULSE_RANGE) / 100;
    }
    
    // Convert to duty cycle
    // PWM period at 50Hz = 20000us
    // Duty = (pulse_width / period) * max_duty
    uint32_t period_us = 1000000 / PWM_FREQUENCY;  // 20000us for 50Hz
    uint32_t duty = ((uint32_t)pulseWidth_us * (1 << PWM_RESOLUTION)) / period_us;
    
    // Set duty cycle
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  }
#endif
