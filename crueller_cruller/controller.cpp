#include "controller.h"
#include "radius_tuner.h"
#include "led.h"
#include "accelerometer.h"
#include "meltybrain.h"
#include <Preferences.h>
#include <math.h>
#include <Arduino.h>

// ============================================================================
// CONTROLLER STATE VARIABLES
// ============================================================================

ControllerPtr myController = nullptr;
bool controllerConnected = false;

// Input state
uint8_t lastDpadState = 0;
float targetDirection = 0;
float translationStrength = 0.0f;
uint16_t currentThrottle = 0;
float translationTrim = TRANSLATION_TRIM_DEFAULT;

// Config mode state
bool configMode = false;
unsigned long lastCalibrationFlash = 0;
bool showCalibrationFlash = false;
bool recalibrateRequested = false;

// ============================================================================
// BLUEPAD32 CALLBACKS
// ============================================================================

void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Serial.printf("CALLBACK: Controller connected, idx=%d\n", ctl->index());
        myController = ctl;
        controllerConnected = true;
        BP32.enableNewBluetoothConnections(false);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.printf("CALLBACK: Controller disconnected, idx=%d\n", ctl->index());
        myController = nullptr;
        controllerConnected = false;
        BP32.enableNewBluetoothConnections(true);
    }
}

// ============================================================================
// CONTROLLER PROCESSING
// ============================================================================

void processController(RadiusTuner& radiusTuner, LEDController& leds) {
    if (myController == nullptr || !myController->isConnected()) {
        return;
    }
    
    uint8_t dpad = myController->dpad();
    int16_t leftStickX = myController->axisX();
    int16_t leftStickY = myController->axisY();

    float stickX_norm = (float)leftStickX / STICK_MAX;
    float stickY_norm = (float)leftStickY / STICK_MAX;
    float stickMagnitude = sqrt(stickX_norm * stickX_norm + stickY_norm * stickY_norm);
    if (stickMagnitude > 1.0f) stickMagnitude = 1.0f;
    float deadzone_norm = (float)LEFT_STICK_DEADZONE / STICK_MAX;
    if (stickMagnitude < deadzone_norm) {
        stickMagnitude = 0.0f;
    } else {
        stickMagnitude = (stickMagnitude - deadzone_norm) / (1.0f - deadzone_norm);
    }

    float stickAngle_rad = atan2(-stickY_norm, stickX_norm);
    float stickAngle_deg = stickAngle_rad * 180.0f / M_PI;
    if (stickAngle_deg < 0) stickAngle_deg += 360.0f;

    if (dpad != 0) {
        translationStrength = 1.0f;
        if (dpad == 1) targetDirection = 0;
        else if (dpad == 2) targetDirection = 90;
        else if (dpad == 4) targetDirection = 180;
        else if (dpad == 8) targetDirection = 270;
        lastDpadState = dpad;
    } else if (stickMagnitude > 0.0f) {
        targetDirection = stickAngle_deg;
        translationStrength = stickMagnitude;
        lastDpadState = 0;
    } else {
        translationStrength = 0.0f;
        lastDpadState = 0;
    }

    int16_t trigger = myController->throttle();
    if (trigger == 0) {
        int16_t rightStickY = myController->axisRY();
        if (rightStickY > 0) trigger = rightStickY;
    }
    currentThrottle = (trigger > 0) ? map(trigger, 0, 512, 0, 100) : 0;
    if (currentThrottle > 100) currentThrottle = 100;
    if (configMode && currentThrottle > CONFIG_MODE_MAX_THROTTLE) {
        currentThrottle = CONFIG_MODE_MAX_THROTTLE;
    }

    // Config mode toggle
    static bool xButtonHeld = false;
    static unsigned long xButtonHoldStart = 0;
    static bool xButtonToggled = false;
    static unsigned long lastToggleTime = 0;
    if (myController->x()) {
        if (!xButtonHeld) {
            xButtonHeld = true;
            xButtonHoldStart = millis();
            xButtonToggled = false;
        } else if (!xButtonToggled && millis() - xButtonHoldStart >= CONFIG_MODE_HOLD_MS) {
            if (millis() - lastToggleTime >= BUTTON_DEBOUNCE_MS) {
                configMode = !configMode;
                xButtonToggled = true;
                lastToggleTime = millis();
                showCalibrationFlash = false;
                if (!configMode) {
                    radiusTuner.save();
                    Preferences prefs;
                    prefs.begin("cruller", false);
                    prefs.putFloat("translation_trim", translationTrim);
                    prefs.end();
                    Serial.printf("Saved modulation strength to EEPROM: %.2f (%.0f%%)\n", translationTrim, translationTrim * 100.0f);
                    resetRotationTracking();
                    showCalibrationFlash = true;
                    lastCalibrationFlash = millis();
                }
                leds.setColor(configMode ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
                Serial.printf("Config mode %s\n", configMode ? "ENABLED" : "DISABLED");
            }
        }
    } else {
        xButtonHeld = false;
        xButtonToggled = false;
    }

    radiusTuner.processStickInput(myController->axisRX());

    // Modulation strength adjustment
    static int16_t lastRightStickY = 0;
    static unsigned long lastModulationUpdate = 0;
    int16_t rightStickY = myController->axisRY();
    if (abs(rightStickY) > MODULATION_DEADZONE && (millis() - lastModulationUpdate > MODULATION_UPDATE_INTERVAL_MS)) {
        lastModulationUpdate = millis();
        float modulation_change = (rightStickY < -MODULATION_DEADZONE) ? -0.01f :
                                  (rightStickY > MODULATION_DEADZONE) ? 0.01f : 0.0f;
        if (modulation_change != 0.0f) {
            translationTrim += modulation_change;
            if (translationTrim < 0.0f) translationTrim = 0.0f;
            if (translationTrim > 2.0f) translationTrim = 2.0f;
            Serial.printf("Modulation strength: %.2f (%.0f%%)\n", translationTrim, translationTrim * 100.0f);
        }
    }
    lastRightStickY = rightStickY;

    if (configMode) {
        static bool lastAButton = false;
        if (myController->a() && !lastAButton) {
            showCalibrationFlash = true;
            lastCalibrationFlash = millis();
        }
        lastAButton = myController->a();
    }

    // Recalibration
    static bool yButtonHeld = false;
    static unsigned long yButtonHoldStart = 0;
    if (currentThrottle == 0 && myController->y()) {
        if (!yButtonHeld) {
            yButtonHeld = true;
            yButtonHoldStart = millis();
        } else if (millis() - yButtonHoldStart >= RECALIBRATE_HOLD_MS) {
            recalibrateRequested = true;
            yButtonHeld = false;
        }
    } else {
        yButtonHeld = false;
    }
}

