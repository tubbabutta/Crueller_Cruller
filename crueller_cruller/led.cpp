#include "led.h"

// NeoPixel configuration
#define NEOPIXEL_PIN 23
#define NUM_PIXELS 2
#define NEOPIXEL_BRIGHTNESS 50

// Global LED controller instance
LEDController leds;

LEDController::LEDController() 
  : pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800) {
}

void LEDController::begin() {
  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
  off();  // Start with LEDs off
}

void LEDController::setColor(LEDColor color) {
  uint32_t colorValue = 0;
  
  switch (color) {
    case LED_COLOR_DISCONNECTED:
      colorValue = pixels.Color(0, 255, 0);  // Red (GRB order)
      break;
    case LED_COLOR_READY:
      colorValue = pixels.Color(0, 0, 255);  // Blue
      break;
    case LED_COLOR_MELTYBRAIN:
      colorValue = pixels.Color(255, 0, 0);  // Green (GRB order)
      break;
    case LED_COLOR_CONFIG_MODE:
      colorValue = pixels.Color(255, 0, 255);  // Purple
      break;
    case LED_COLOR_CALIBRATED:
      colorValue = pixels.Color(255, 255, 0);  // Yellow
      break;
    case LED_COLOR_OFF:
    default:
      colorValue = 0;
      break;
  }
  
  setColor(colorValue);
}

void LEDController::setColor(uint32_t color) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void LEDController::off() {
  setColor(0);
}

