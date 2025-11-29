#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include "config.h"

// Forward declaration
class RadiusTuner;

// ============================================================================
// ACCELEROMETER CONFIGURATION
// ============================================================================
// Parameters for accelerometer calibration and RPM calculation

// Accelerometer calibration
const int CALIBRATION_SAMPLES = 60;  // Number of samples to average during calibration

// RPM calculation constants
const float RPM_CALCULATION_CONSTANT = 89445.0f;  // Constant for converting acceleration to RPM
const float ACCEL_CORRECTION_FACTOR = 1.0f;        // Multiplier to correct RPM calculation (tune if RPM is consistently off)

// Rotation interval smoothing (for handling rapid RPM changes during battle)
const float SMOOTHING_FAST = 0.3f;        // Fast smoothing factor for large RPM changes (>15%)
const float SMOOTHING_SLOW = 0.1f;        // Slow smoothing factor for small RPM changes (<15%)
const float CHANGE_THRESHOLD = 0.15f;     // Threshold for determining fast vs slow smoothing (15% change)

// ============================================================================
// ACCELEROMETER STATE VARIABLES
// ============================================================================
// Global state variables (declared in accelerometer.cpp, extern here)

extern Adafruit_H3LIS331 lis;           // Accelerometer sensor object
extern bool accelEnabled;                // Whether accelerometer is initialized and working
extern float lastAccelX, lastAccelY, lastAccelZ;  // Last read acceleration values (g-force, offset-corrected)
extern float accelX_offset, accelY_offset, accelZ_offset;  // Calibration offsets (g-force)

// Rotation detection state
extern bool isRotating;                  // Whether robot is currently rotating
extern unsigned long rotationDetectedTime;  // Timestamp when rotation was first detected
extern float current_rpm;                // Current calculated RPM
extern float accel_correction_factor;    // Correction factor for RPM calculation

// Rotation interval smoothing state
extern float smoothed_rotation_interval_ms;  // Smoothed rotation interval (ms)
extern unsigned long last_interval_update;   // Timestamp of last interval update

// ============================================================================
// ACCELEROMETER FUNCTIONS
// ============================================================================

// Initialize accelerometer (call once in setup)
bool initAccelerometer();

// Read current acceleration values (updates lastAccelX, lastAccelY, lastAccelZ)
void readAccelerometer();

// Recalibrate accelerometer offsets (robot must be at rest)
void recalibrateAccelerometer();

// Get horizontal acceleration magnitude in g-force
float getAccelForceG();

// Calculate current RPM from accelerometer data
float calculateRPM(RadiusTuner& radiusTuner);

// Get smoothed rotation interval in milliseconds
float getRotationIntervalMs(RadiusTuner& radiusTuner);

#endif // ACCELEROMETER_H

