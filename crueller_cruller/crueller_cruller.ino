// ============================================================================
// ESP32 MINI MELTYBRAIN ROBOT CONTROLLER
// ============================================================================
// Main program file - coordinates all subsystems

#include "config.h"
#include "dshot.h"
#include "led.h"
#include "radius_tuner.h"
#include "accelerometer.h"
#include "meltybrain.h"
#include "controller.h"
#include <Bluepad32.h>
#include <Preferences.h>
#include <Arduino.h>

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

DShotRMT escMotor((gpio_num_t)ESC_PIN, (rmt_channel_t)ESC_RMT_CHANNEL);
LEDController leds;
RadiusTuner radiusTuner(2.4f);  // Default: 2.4cm (24mm)

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Mini Meltybrain Robot Controller");
    Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());

    leds.begin();
    Serial.println("Initializing ESC DShot600...");
    escMotor.begin(DSHOT600);
    Serial.println("ESC DShot600 initialized - starting arming sequence...");

    for (int i = 0; i < 2000; i++) {
        escMotor.sendThrottleValue(0);
        delay(1);
    }
    Serial.println("ESC arming sequence complete - ESC should be armed");

    radiusTuner.begin();

    Preferences prefs;
    prefs.begin("cruller", true);
    float saved_trim = prefs.getFloat("translation_trim", TRANSLATION_TRIM_DEFAULT);
    prefs.end();
    if (saved_trim >= 0.0f && saved_trim <= 2.0f) {
        translationTrim = saved_trim;
        Serial.printf("Loaded saved modulation strength: %.2f (%.0f%%)\n", translationTrim, translationTrim * 100.0f);
    } else {
        translationTrim = TRANSLATION_TRIM_DEFAULT;
        Serial.printf("Using default modulation strength: %.2f (%.0f%%)\n", translationTrim, translationTrim * 100.0f);
    }

    unsigned long startupStart = millis();
    bool blinkState = false;
    while (millis() - startupStart < 3000) {
        blinkState = !blinkState;
        leds.setColor(blinkState ? LED_COLOR_DISCONNECTED : LED_COLOR_OFF);
        escMotor.sendThrottleValue(0);
        delay(250);
    }
    leds.setColor(LED_COLOR_DISCONNECTED);
    Serial.println("Startup complete - ESC should be armed");

    initAccelerometer();

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableNewBluetoothConnections(true);
    Serial.println("Bluepad32 initialized - waiting for Stadia controller...");
    Serial.println("Put Stadia controller in pairing mode (hold Stadia button for 3 seconds)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processController(radiusTuner, leds);
    }

    if (recalibrateRequested && currentThrottle == 0) {
        recalibrateAccelerometer();
        recalibrateRequested = false;
    }

    if (showCalibrationFlash) {
        unsigned long flashTime = millis() - lastCalibrationFlash;
        if (flashTime < 100) {
            leds.setColor(LED_COLOR_CALIBRATED);
        } else if (flashTime < 200) {
            leds.off();
        } else if (flashTime < 300) {
            leds.setColor(LED_COLOR_CALIBRATED);
        } else if (flashTime < 400) {
            leds.off();
        } else {
            showCalibrationFlash = false;
            if (configMode) {
                leds.setColor(LED_COLOR_CONFIG_MODE);
            }
        }
    }

    bool disconnected = !controllerConnected ||
                        (myController == nullptr) ||
                        (myController != nullptr && !myController->isConnected());

    if (disconnected) {
        leds.setColor(LED_COLOR_DISCONNECTED);
        escMotor.sendThrottleValue(0);
        delay(1);
        return;
    }

    if (currentThrottle > 0) {
        spin_one_rotation(
            radiusTuner,
            leds,
            escMotor,
            currentThrottle,
            translationStrength,
            translationTrim,
            targetDirection,
            configMode,
            showCalibrationFlash,
            controllerConnected
        );
    } else {
        escMotor.sendThrottleValue(0);
        static unsigned long lastLEDUpdate = 0;
        if (millis() - lastLEDUpdate > 100) {
            if (configMode && !showCalibrationFlash) {
                leds.setColor(LED_COLOR_CONFIG_MODE);
            } else if (!showCalibrationFlash) {
                leds.setColor(LED_COLOR_READY);
            }
            lastLEDUpdate = millis();
        }
        delay(1);
    }
}
