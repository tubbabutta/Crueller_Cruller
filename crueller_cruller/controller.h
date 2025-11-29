#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Bluepad32.h>
#include "config.h"

// Forward declarations
class RadiusTuner;
class LEDController;

// ============================================================================
// CONTROLLER INPUT CONFIGURATION
// ============================================================================
// Parameters for processing controller input

// Stick deadzones and scaling
const int16_t LEFT_STICK_DEADZONE = 50;        // Deadzone for left stick (prevents drift)
const float STICK_MAX = 512.0f;                 // Maximum stick value from controller

// Modulation strength adjustment
const int16_t MODULATION_DEADZONE = 100;        // Deadzone for right stick Y axis
const unsigned long MODULATION_UPDATE_INTERVAL_MS = 100;  // Minimum time between modulation updates

// Button hold timings
const unsigned long CONFIG_MODE_HOLD_MS = 2000;  // Hold X button for 2 seconds to toggle config mode
const unsigned long RECALIBRATE_HOLD_MS = 1000;  // Hold Y button for 1 second to recalibrate
const unsigned long BUTTON_DEBOUNCE_MS = 500;    // Minimum time between button toggles

// Config mode throttle limit
const uint16_t CONFIG_MODE_MAX_THROTTLE = 50;   // Maximum throttle in config mode (safety limit)

// ============================================================================
// CONTROLLER STATE VARIABLES
// ============================================================================
// Global state variables (declared in controller.cpp, extern here)

extern ControllerPtr myController;              // Pointer to connected controller
extern bool controllerConnected;                // Whether a controller is connected

// Input state
extern uint8_t lastDpadState;                   // Last D-pad state (for change detection)
extern float targetDirection;                   // Target translation direction (0-360 degrees)
extern float translationStrength;               // Translation strength (0.0-1.0, from stick magnitude)
extern uint16_t currentThrottle;                // Current throttle percentage (0-100)
extern float translationTrim;                   // Modulation strength multiplier (0.0-2.0, 1.0 = 100%)

// Config mode state
extern bool configMode;                         // Whether config/calibration mode is active
extern unsigned long lastCalibrationFlash;     // Timestamp of last calibration flash
extern bool showCalibrationFlash;               // Whether to show calibration saved flash
extern bool recalibrateRequested;               // Whether accelerometer recalibration was requested

// ============================================================================
// CONTROLLER FUNCTIONS
// ============================================================================

// Process controller input and update state variables
void processController(
    RadiusTuner& radiusTuner,
    LEDController& leds
);

// Bluepad32 callback functions
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);

#endif // CONTROLLER_H

