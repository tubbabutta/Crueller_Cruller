#ifndef RADIUS_TUNER_H
#define RADIUS_TUNER_H

#include <Arduino.h>

// ============================================================================
// RADIUS TUNER CONFIGURATION
// ============================================================================
// Parameters for adjusting accelerometer radius calibration

// Radius limits (in centimeters)
const float RADIUS_MIN_CM = 0.5f;      // Minimum allowed radius (0.5cm = 5mm)
const float RADIUS_MAX_CM = 5.0f;       // Maximum allowed radius (5.0cm = 50mm)
const float RADIUS_INCREMENT_CM = 0.001f;  // Radius adjustment step size (0.001cm = 0.01mm per stick movement)

// Stick input processing
const int16_t STICK_DEADZONE = 100;           // Deadzone for stick input (prevents drift)
const unsigned long UPDATE_INTERVAL_MS = 12;  // Minimum time between radius updates (prevents too-fast changes)
const unsigned long FLASH_INTERVAL_MS = 200;  // Time between visual feedback flashes

// ============================================================================
// RADIUS TUNER CLASS
// ============================================================================
// Manages accelerometer radius calibration for accurate RPM calculation.
// The radius is the distance from the center of rotation to the accelerometer.
// This value directly affects RPM calculation accuracy.

class RadiusTuner {
public:
    RadiusTuner(float default_radius_cm);
    void begin();
    bool processStickInput(int16_t stickX);
    float getRadius() const;
    void setRadius(float radius_cm);
    void save();
    void clear();

private:
    float current_radius_cm;        // Current radius value (cm)
    float default_radius_cm;        // Default radius if no saved value exists
    float min_radius_cm;            // Minimum allowed radius (cm)
    float max_radius_cm;            // Maximum allowed radius (cm)
    float radius_increment_cm;      // Step size for radius adjustments (cm)
    int16_t lastStickX;             // Last stick X position (for change detection)
    unsigned long lastRadiusUpdate; // Timestamp of last radius update
    unsigned long lastRadiusFlash;  // Timestamp of last visual feedback flash
};

#endif // RADIUS_TUNER_H

