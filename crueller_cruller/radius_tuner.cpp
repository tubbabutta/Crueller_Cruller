#include "radius_tuner.h"
#include <Preferences.h>

RadiusTuner::RadiusTuner(float default_radius_cm)
    : current_radius_cm(default_radius_cm),
      default_radius_cm(default_radius_cm),
      min_radius_cm(RADIUS_MIN_CM),
      max_radius_cm(RADIUS_MAX_CM),
      radius_increment_cm(RADIUS_INCREMENT_CM),
      lastStickX(0),
      lastRadiusUpdate(0),
      lastRadiusFlash(0) {}

void RadiusTuner::begin() {
    Preferences preferences;
    preferences.begin("cruller", true);
    float saved_radius = preferences.getFloat("radius_cm", default_radius_cm);
    preferences.end();

    if (saved_radius >= min_radius_cm && saved_radius <= max_radius_cm) {
        current_radius_cm = saved_radius;
        Serial.printf("Loaded saved radius from EEPROM: %.2f cm (%.1f mm)\n",
                      saved_radius, saved_radius * 10.0f);
    } else {
        current_radius_cm = default_radius_cm;
        Serial.printf("Using default radius: %.2f cm (%.1f mm) (no valid saved value)\n",
                      default_radius_cm, default_radius_cm * 10.0f);
    }
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
                lastRadiusFlash = millis();
                return true;
            }
        }
    }
    lastStickX = stickX;
    return false;
}

float RadiusTuner::getRadius() const {
    return current_radius_cm;
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

void RadiusTuner::save() {
    Preferences preferences;
    preferences.begin("cruller", false);
    preferences.putFloat("radius_cm", current_radius_cm);
    preferences.end();
    Serial.printf("Saved radius to EEPROM: %.2f cm (%.1f mm)\n",
                  current_radius_cm, current_radius_cm * 10.0f);
}

void RadiusTuner::clear() {
    Preferences preferences;
    preferences.begin("cruller", false);
    preferences.remove("radius_cm");
    preferences.end();
    Serial.println("Cleared saved radius from EEPROM - will use default on next boot");
}

