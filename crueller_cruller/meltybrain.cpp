#include "meltybrain.h"
#include "accelerometer.h"
#include "dshot.h"
#include "led.h"
#include "radius_tuner.h"
#include <math.h>
#include <Arduino.h>

// ============================================================================
// MELTYBRAIN STATE VARIABLES
// ============================================================================

unsigned long rotation_started_at_us = 0;

// ============================================================================
// MELTYBRAIN CONTROL FUNCTIONS
// ============================================================================

struct melty_parameters_t getMeltyParameters(
    RadiusTuner& radiusTuner,
    uint16_t currentThrottle,
    float translationStrength,
    float translationTrim,
    float targetDirection
) {
    struct melty_parameters_t params = {};
    params.throttle_percent = currentThrottle / 100.0f;
    current_rpm = calculateRPM(radiusTuner);
    float rotation_interval_ms = getRotationIntervalMs(radiusTuner);
    params.rotation_interval_us = (unsigned long)(rotation_interval_ms * 1000);
    if (params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US) {
        params.rotation_interval_us = MAX_TRANSLATION_ROTATION_INTERVAL_US;
    }
    bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                              params.rotation_interval_us > 0 && 
                              params.throttle_percent > 0);
    params.target_direction_degrees = targetDirection;
    if (meltybrain_active && translationStrength > 0) {
        params.max_throttle_offset = (float)currentThrottle * translationStrength * translationTrim;
        if (params.max_throttle_offset > currentThrottle * MODULATION_LIMIT_PERCENT) {
            params.max_throttle_offset = currentThrottle * MODULATION_LIMIT_PERCENT;
        }
    } else {
        params.max_throttle_offset = 0;
    }
    
    // Fixed LED on portion: 10% of rotation
    unsigned long led_on_us = (unsigned long)(LED_ON_PORTION * params.rotation_interval_us);
    float base_offset_percent = LED_OFFSET_PERCENT;
    float translation_offset_percent = 0.0f;
    if (params.max_throttle_offset > 0 && translationStrength > 0) {
        translation_offset_percent = params.target_direction_degrees / 360.0f * 100.0f;
    }
    float led_offset_percent = base_offset_percent + translation_offset_percent;
    if (led_offset_percent >= 100.0f) led_offset_percent -= 100.0f;
    unsigned long led_offset_us = (unsigned long)(led_offset_percent * params.rotation_interval_us / 100);
    long led_start = led_offset_us - (led_on_us / 2);
    if (led_start < 0) led_start += params.rotation_interval_us;
    long led_stop = led_start + led_on_us;
    if (led_stop >= (long)params.rotation_interval_us) led_stop -= params.rotation_interval_us;
    params.led_start_us = (unsigned long)led_start;
    params.led_stop_us = (unsigned long)led_stop;
    return params;
}

void updateHeadingLED(
    struct melty_parameters_t* params,
    unsigned long time_spent_us,
    LEDController& leds,
    bool configMode,
    bool showCalibrationFlash,
    float current_rpm
) {
    bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                             params->rotation_interval_us > 0 && 
                             params->rotation_interval_us <= MAX_TRANSLATION_ROTATION_INTERVAL_US);
    if (!meltybrain_active) {
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
        return;
    }
    bool led_on;
    if (params->led_start_us > params->led_stop_us) {
        led_on = (time_spent_us >= params->led_start_us) || (time_spent_us <= params->led_stop_us);
    } else {
        led_on = (time_spent_us >= params->led_start_us && time_spent_us <= params->led_stop_us);
    }
    if (led_on) {
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_MELTYBRAIN);
    } else {
        leds.off();
    }
}

void updateMotorTranslation(
    struct melty_parameters_t* params,
    unsigned long time_spent_us,
    DShotRMT& escMotor,
    uint16_t currentThrottle,
    float current_rpm
) {
    if (params->rotation_interval_us == 0 || params->throttle_percent == 0) {
        escMotor.sendThrottleValue(0);
        return;
    }
    if (current_rpm < MELTYBRAIN_RPM_THRESHOLD || params->max_throttle_offset == 0) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        return;
    }
    
    // Parabolic sine wave approximation for throttle modulation
    long micros_into_phase = time_spent_us % (params->rotation_interval_us / 2);
    float phase_progress = 2.0f * micros_into_phase / (float)params->rotation_interval_us;
    float phase_offset_fraction = -4.0f * phase_progress * (phase_progress - 1.0f);
    float base_throttle_offset = phase_offset_fraction * params->max_throttle_offset;
    float current_phase = (float)time_spent_us / (float)params->rotation_interval_us * 2.0f * M_PI;
    while (current_phase >= 2.0f * M_PI) current_phase -= 2.0f * M_PI;
    while (current_phase < 0) current_phase += 2.0f * M_PI;
    float target_phase = params->target_direction_degrees * M_PI / 180.0f;
    float phase_diff = current_phase - target_phase;
    while (phase_diff > M_PI) phase_diff -= 2.0f * M_PI;
    while (phase_diff < -M_PI) phase_diff += 2.0f * M_PI;
    float throttle_offset;
    if (abs(phase_diff) < M_PI / 2.0f) {
        throttle_offset = base_throttle_offset;
    } else {
        throttle_offset = -base_throttle_offset;
    }
    float final_throttle = (currentThrottle / 100.0f) + (throttle_offset / 100.0f);
    if (final_throttle < 0) final_throttle = 0;
    if (final_throttle > 1.0) final_throttle = 1.0;
    escMotor.sendThrottleValue(throttlePercentToDShot((uint16_t)(final_throttle * 100)));
}

void spin_one_rotation(
    RadiusTuner& radiusTuner,
    LEDController& leds,
    DShotRMT& escMotor,
    uint16_t currentThrottle,
    float translationStrength,
    float translationTrim,
    float targetDirection,
    bool configMode,
    bool showCalibrationFlash,
    bool controllerConnected
) {
    if (!accelEnabled) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
        return;
    }
    
    readAccelerometer();
    float horizontalAccel = sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY);
    unsigned long currentTime = millis();
    if (horizontalAccel > ROTATION_THRESHOLD) {
        if (!isRotating) {
            if (rotationDetectedTime == 0) {
                rotationDetectedTime = currentTime;
            } else if (currentTime - rotationDetectedTime >= ROTATION_DEBOUNCE_MS) {
                isRotating = true;
            }
        }
    } else {
        rotationDetectedTime = 0;
        isRotating = false;
    }
    
    struct melty_parameters_t melty_params = getMeltyParameters(radiusTuner, currentThrottle, translationStrength, translationTrim, targetDirection);
    if (!isRotating || melty_params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US || melty_params.rotation_interval_us == 0) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        if (configMode && !showCalibrationFlash) {
            leds.setColor(LED_COLOR_CONFIG_MODE);
        } else if (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && melty_params.rotation_interval_us > 0) {
            leds.setColor(LED_COLOR_MELTYBRAIN);
        } else {
            leds.setColor(LED_COLOR_READY);
        }
        return;
    }
    
    if (melty_params.rotation_interval_us > 1000000) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
        return;
    }
    
    // Initialize rotation tracking
    if (rotation_started_at_us == 0) {
        rotation_started_at_us = micros();
    }

    long time_spent_this_rotation_us = micros() - rotation_started_at_us;
    static unsigned long last_rotation_interval_us = 0;
    if (last_rotation_interval_us > 0) {
        float interval_change_ratio = fabs((long)melty_params.rotation_interval_us - (long)last_rotation_interval_us) / (float)last_rotation_interval_us;
        if (interval_change_ratio > 0.2f) {
            rotation_started_at_us = micros();
            time_spent_this_rotation_us = 0;
        }
    }
    last_rotation_interval_us = melty_params.rotation_interval_us;

    if (time_spent_this_rotation_us > (long)melty_params.rotation_interval_us) {
        time_spent_this_rotation_us -= melty_params.rotation_interval_us;
        rotation_started_at_us += melty_params.rotation_interval_us;
    }

    updateMotorTranslation(&melty_params, time_spent_this_rotation_us, escMotor, currentThrottle, current_rpm);

    // Blocking loop for one rotation - resilient to RPM changes
    while (true) {
        time_spent_this_rotation_us = micros() - rotation_started_at_us;

        // Check for RPM changes every 10ms
        static unsigned long last_params_update = 0;
        if (micros() - last_params_update > PARAMS_UPDATE_INTERVAL_US) {
            struct melty_parameters_t new_params = getMeltyParameters(radiusTuner, currentThrottle, translationStrength, translationTrim, targetDirection);
            float interval_change = fabs((long)new_params.rotation_interval_us - (long)melty_params.rotation_interval_us) / (float)melty_params.rotation_interval_us;
            if (interval_change > INTERVAL_CHANGE_THRESHOLD) {
                melty_params = new_params;
                if (interval_change > INTERVAL_CHANGE_RESET) {
                    rotation_started_at_us = micros();
                    time_spent_this_rotation_us = 0;
                }
                last_rotation_interval_us = melty_params.rotation_interval_us;
            }
            last_params_update = micros();
        }

        if (time_spent_this_rotation_us > (long)melty_params.rotation_interval_us) {
            time_spent_this_rotation_us -= melty_params.rotation_interval_us;
            rotation_started_at_us += melty_params.rotation_interval_us;
            melty_params = getMeltyParameters(radiusTuner, currentThrottle, translationStrength, translationTrim, targetDirection);
            last_rotation_interval_us = melty_params.rotation_interval_us;
        }

        updateMotorTranslation(&melty_params, time_spent_this_rotation_us, escMotor, currentThrottle, current_rpm);

        // Note: Controller updates are handled in main loop via BP32.update()

        if (currentThrottle == 0) {
            escMotor.sendThrottleValue(0);
            leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
            break;
        }

        if (!controllerConnected) {
            escMotor.sendThrottleValue(0);
            leds.setColor(LED_COLOR_DISCONNECTED);
            break;
        }

        updateHeadingLED(&melty_params, time_spent_this_rotation_us, leds, configMode, showCalibrationFlash, current_rpm);
        delayMicroseconds(10);
    }

    if (currentThrottle == 0) {
        rotation_started_at_us = 0;
    }
}

void resetRotationTracking() {
    rotation_started_at_us = 0;
}

