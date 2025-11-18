#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// LED status colors
enum LEDColor {
  LED_COLOR_DISCONNECTED,  // Red
  LED_COLOR_READY,         // Blue
  LED_COLOR_MELTYBRAIN,    // Green
  LED_COLOR_CONFIG_MODE,   // Purple
  LED_COLOR_CALIBRATED,    // Yellow
  LED_COLOR_OFF            // Off
};

class LEDController {
public:
  LEDController();
  void begin();
  void setColor(LEDColor color);
  void setColor(uint32_t color);  // Direct color value (GRB format)
  void off();
  
private:
  Adafruit_NeoPixel pixels;
  void updateLEDs();
};

// Global LED controller instance
extern LEDController leds;

#endif // LED_H

