#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>
#include "config.h"

// ============================================================================
// LED COLOR DEFINITIONS
// ============================================================================
// Visual feedback states for robot status

enum LEDColor {
    LED_COLOR_DISCONNECTED,  // Red - No controller connected
    LED_COLOR_READY,         // Blue - Controller connected, ready to operate
    LED_COLOR_MELTYBRAIN,     // Green - Meltybrain translation active
    LED_COLOR_CONFIG_MODE,    // Purple - Configuration/calibration mode
    LED_COLOR_CALIBRATED,     // Yellow - Calibration saved confirmation
    LED_COLOR_OFF             // Off - LEDs disabled
};

// ============================================================================
// LED CONTROLLER CLASS
// ============================================================================
// Manages NeoPixel LED strip for visual feedback

class LEDController {
public:
    LEDController();
    void begin();
    void setColor(LEDColor color);
    void setColor(uint32_t color);
    void off();

private:
    Adafruit_NeoPixel pixels;  // NeoPixel strip object
};

#endif // LED_H

