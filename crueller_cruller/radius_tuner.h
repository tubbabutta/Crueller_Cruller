#ifndef RADIUS_TUNER_H
#define RADIUS_TUNER_H

#include <Arduino.h>

/**
 * Real-time radius adjustment for POV LED stability
 * Right stick X: left = decrease, right = increase (0.1mm steps)
 */
class RadiusTuner {
public:
  RadiusTuner(float default_radius_cm);
  
  void begin();
  bool processStickInput(int16_t stickX);  // Returns true if adjusted
  float getRadius() const { return current_radius_cm; }
  void reset();
  void setRadius(float radius_cm);

private:
  float current_radius_cm;
  float default_radius_cm;
  float min_radius_cm;
  float max_radius_cm;
  float radius_increment_cm;
  
  int16_t lastStickX;
  unsigned long lastRadiusUpdate;
  unsigned long lastRadiusFlash;
  
  static const int16_t STICK_DEADZONE = 100;
  static const unsigned long UPDATE_INTERVAL_MS = 50;  // 20 steps per second
  static const unsigned long FLASH_INTERVAL_MS = 200;  // Flash every 200ms while adjusting
};

// Global radius tuner instance
extern RadiusTuner radiusTuner;

#endif // RADIUS_TUNER_H

