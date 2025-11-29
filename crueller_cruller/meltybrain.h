#ifndef MELTYBRAIN_H
#define MELTYBRAIN_H

#include <stdint.h>
#include "config.h"

// Forward declarations
class RadiusTuner;
class LEDController;
class DShotRMT;

// ============================================================================
// MELTYBRAIN CONTROL PARAMETERS
// ============================================================================
// Parameters that control translation behavior and modulation

// Modulation limits
const float MODULATION_LIMIT_PERCENT = 0.7f;  // Maximum throttle modulation (70% of base throttle)

// LED timing
const float LED_ON_PORTION = 0.10f;  // Percentage of rotation that LED is on (10% = narrow beam)

// RPM change detection for phase reset
const float INTERVAL_CHANGE_THRESHOLD = 0.15f;  // 15% change triggers parameter update
const float INTERVAL_CHANGE_RESET = 0.3f;        // 30% change triggers phase reset

// Rotation tracking update intervals
const unsigned long PARAMS_UPDATE_INTERVAL_US = 10000;  // Update parameters every 10ms
const unsigned long CONTROLLER_CHECK_INTERVAL_US = 200; // Check controller every 200us

// ============================================================================
// MELTYBRAIN PARAMETERS STRUCTURE
// ============================================================================
// Contains all parameters needed for one rotation cycle

struct melty_parameters_t {
    float throttle_percent;           // Base throttle percentage (0.0-1.0)
    unsigned long rotation_interval_us; // Time for one complete rotation (microseconds)
    unsigned long led_start_us;        // LED on start time within rotation (microseconds)
    unsigned long led_stop_us;         // LED off time within rotation (microseconds)
    float target_direction_degrees;    // Target translation direction (0-360 degrees)
    float max_throttle_offset;         // Maximum throttle modulation amount (0.0-1.0)
};

// ============================================================================
// MELTYBRAIN STATE VARIABLES
// ============================================================================
// Global state variables (declared in meltybrain.cpp, extern here)

extern unsigned long rotation_started_at_us;  // Timestamp when current rotation started (for phase tracking)

// ============================================================================
// MELTYBRAIN CONTROL FUNCTIONS
// ============================================================================

// Calculate meltybrain parameters for current state
struct melty_parameters_t getMeltyParameters(
    RadiusTuner& radiusTuner,
    uint16_t currentThrottle,
    float translationStrength,
    float translationTrim,
    float targetDirection
);

// Update LED based on rotation phase
void updateHeadingLED(
    struct melty_parameters_t* params,
    unsigned long time_spent_us,
    LEDController& leds,
    bool configMode,
    bool showCalibrationFlash,
    float current_rpm
);

// Update motor throttle with modulation for translation
void updateMotorTranslation(
    struct melty_parameters_t* params,
    unsigned long time_spent_us,
    DShotRMT& escMotor,
    uint16_t currentThrottle,
    float current_rpm
);

// Execute one complete rotation cycle with translation
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
);

// Reset rotation tracking (call when exiting config mode or recalibrating)
void resetRotationTracking();

#endif // MELTYBRAIN_H

