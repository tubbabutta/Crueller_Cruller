#include "radius_tuner.h"
#include "led.h"

// Global radius tuner instance
RadiusTuner radiusTuner(3.4f);  // Default: 3.4cm (34mm)

RadiusTuner::RadiusTuner(float default_radius_cm)
  : current_radius_cm(default_radius_cm),
    default_radius_cm(default_radius_cm),
    min_radius_cm(1.0f),      // 10mm (minimum reasonable radius)
    max_radius_cm(3.6f),       // 36mm (half of 72mm diameter)
    radius_increment_cm(0.01f), // 0.1mm per step (fine adjustment)
    lastStickX(0),
    lastRadiusUpdate(0),
    lastRadiusFlash(0)
{
}

void RadiusTuner::begin() {
  current_radius_cm = default_radius_cm;
  lastStickX = 0;
  lastRadiusUpdate = 0;
  lastRadiusFlash = 0;
}

bool RadiusTuner::processStickInput(int16_t stickX) {
  if (abs(stickX) > STICK_DEADZONE && (millis() - lastRadiusUpdate > UPDATE_INTERVAL_MS)) {
    lastRadiusUpdate = millis();
    
    float radius_change = (stickX < -STICK_DEADZONE) ? -radius_increment_cm : 
                          (stickX > STICK_DEADZONE) ? radius_increment_cm : 0.0f;
    
    if (radius_change != 0.0f) {
      float new_radius = current_radius_cm + radius_change;
      if (new_radius < min_radius_cm) new_radius = min_radius_cm;
      if (new_radius > max_radius_cm) new_radius = max_radius_cm;
      
      current_radius_cm = new_radius;
      
      if (millis() - lastRadiusFlash > FLASH_INTERVAL_MS) {
        leds.setColor(LED_COLOR_CONFIG_MODE);
        lastRadiusFlash = millis();
        return true;
      }
    }
  }
  
  lastStickX = stickX;
  return false;
}

void RadiusTuner::reset() {
  current_radius_cm = default_radius_cm;
}

void RadiusTuner::setRadius(float radius_cm) {
  if (radius_cm < min_radius_cm) {
    current_radius_cm = min_radius_cm;
  } else if (radius_cm > max_radius_cm) {
    current_radius_cm = max_radius_cm;
  } else {
    current_radius_cm = radius_cm;
  }
}

