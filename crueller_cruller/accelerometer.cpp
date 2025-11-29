#include "accelerometer.h"
#include "radius_tuner.h"
#include <Wire.h>
#include <math.h>

// ============================================================================
// ACCELEROMETER STATE VARIABLES
// ============================================================================

Adafruit_H3LIS331 lis;
bool accelEnabled = false;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

// Rotation detection state
bool isRotating = false;
unsigned long rotationDetectedTime = 0;
float current_rpm = 0;
float accel_correction_factor = ACCEL_CORRECTION_FACTOR;

// Rotation interval smoothing state
float smoothed_rotation_interval_ms = 0.0f;
unsigned long last_interval_update = 0;

// ============================================================================
// ACCELEROMETER FUNCTIONS
// ============================================================================

bool initAccelerometer() {
    Wire.begin(ACCEL_SDA, ACCEL_SCL);
    bool accelInitialized = lis.begin_I2C();
    if (!accelInitialized) {
        accelInitialized = lis.begin_I2C(0x19);
    }

    if (accelInitialized) {
        Serial.println("H3LIS331DL Accelerometer found!");
        lis.setRange(H3LIS331_RANGE_400_G);  // 400g range for high-g meltybrain application
        lis.setDataRate(LIS331_DATARATE_1000_HZ);
        Serial.println("Calibrating accelerometer offsets (robot must be at rest)...");
        delay(100);
        float sumX = 0, sumY = 0, sumZ = 0;
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            sensors_event_t event;
            if (lis.getEvent(&event)) {
                sumX += event.acceleration.x;
                sumY += event.acceleration.y;
                sumZ += event.acceleration.z;
            }
            delay(1);
        }
        accelX_offset = sumX / CALIBRATION_SAMPLES;
        accelY_offset = sumY / CALIBRATION_SAMPLES;
        accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;
        Serial.print("Calibration complete - X offset: ");
        Serial.print(accelX_offset);
        Serial.print(", Y offset: ");
        Serial.print(accelY_offset);
        Serial.print(", Z offset: ");
        Serial.println(accelZ_offset);
        accelEnabled = true;
        Serial.println("H3LIS331DL initialized for meltybrain");
        return true;
    } else {
        accelEnabled = false;
        Serial.println("H3LIS331DL not found - meltybrain features disabled");
        return false;
    }
}

void readAccelerometer() {
    sensors_event_t event;
    if (lis.getEvent(&event)) {
        lastAccelX = event.acceleration.x - accelX_offset;
        lastAccelY = event.acceleration.y - accelY_offset;
        lastAccelZ = event.acceleration.z - accelZ_offset;
    }
}

void recalibrateAccelerometer() {
    if (!accelEnabled) {
        Serial.println("Cannot recalibrate: accelerometer not enabled");
        return;
    }
    Serial.println("Recalibrating accelerometer offsets (robot must be at rest)...");
    delay(100);
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sensors_event_t event;
        if (lis.getEvent(&event)) {
            sumX += event.acceleration.x;
            sumY += event.acceleration.y;
            sumZ += event.acceleration.z;
        }
        delayMicroseconds(1000);
    }
    accelX_offset = sumX / CALIBRATION_SAMPLES;
    accelY_offset = sumY / CALIBRATION_SAMPLES;
    accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;
    Serial.print("Recalibration complete - X offset: ");
    Serial.print(accelX_offset);
    Serial.print(", Y offset: ");
    Serial.print(accelY_offset);
    Serial.print(", Z offset: ");
    Serial.println(accelZ_offset);
    isRotating = false;
    rotationDetectedTime = 0;
    current_rpm = 0;
}

float getAccelForceG() {
    readAccelerometer();
    return sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY) / 9.81;
}

float calculateRPM(RadiusTuner& radiusTuner) {
    float accel_g = fabs(getAccelForceG());
    if (accel_g <= 0 || isnan(accel_g) || isinf(accel_g)) {
        return 0.0;
    }
    float radius_cm = radiusTuner.getRadius();
    float rpm = accel_g * RPM_CALCULATION_CONSTANT / radius_cm;
    rpm = sqrt(rpm) * accel_correction_factor;
    if (isnan(rpm) || isinf(rpm) || rpm < 0) {
        return 0.0;
    }
    return rpm;
}

float getRotationIntervalMs(RadiusTuner& radiusTuner) {
    float rpm = calculateRPM(radiusTuner);
    if (rpm < MIN_TRACKING_RPM) rpm = MIN_TRACKING_RPM;
    float new_interval_ms = (1.0f / rpm) * 60 * 1000;
    unsigned long now = millis();
    if (smoothed_rotation_interval_ms == 0.0f) {
        smoothed_rotation_interval_ms = new_interval_ms;
        last_interval_update = now;
    } else {
        float change_ratio = fabs(new_interval_ms - smoothed_rotation_interval_ms) / smoothed_rotation_interval_ms;
        float smoothing_factor = (change_ratio > CHANGE_THRESHOLD) ? SMOOTHING_FAST : SMOOTHING_SLOW;
        smoothed_rotation_interval_ms = smoothed_rotation_interval_ms * (1.0f - smoothing_factor) +
                                         new_interval_ms * smoothing_factor;
        last_interval_update = now;
    }
    return smoothed_rotation_interval_ms;
}

