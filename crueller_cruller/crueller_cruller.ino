/*
 * ESP32 Mini Meltybrain Robot Controller
 * 
 * Single-wheel combat robot controller with meltybrain translation control.
 * 
 * Hardware:
 * - Google Stadia controller (Bluetooth Classic via Bluepad32)
 * - Two daisy-chained NeoPixels on pin 23 (status indicators)
 * - AM32 ESC via DShot600 on pin 19 (motor control)
 * - LIS331HH accelerometer on I2C (SDA=21, SCL=22) for rotation tracking
 * 
 * Control Modes:
 * - Normal: Direct throttle control via right trigger
 * - Meltybrain: Active when RPM exceeds threshold, enables directional translation
 *   while spinning using sine wave throttle modulation
 * 
 * LED Status Indicators:
 * - Red (solid): Controller disconnected
 * - Red (blinking): Startup sequence (3 seconds)
 * - Blue (solid): Connected and ready, or spinning up
 * - Green (blinking): Meltybrain mode active, blinks at heading position (POV effect)
 * 
 * Meltybrain Operation:
 * The robot spins at maximum throttle when the trigger is held. Once RPM exceeds
 * the threshold, meltybrain mode activates. In this mode, the motor throttle is
 * modulated with a sine wave pattern synchronized to rotation. By applying the
 * modulation at specific phases of rotation, the robot translates in the direction
 * commanded by the D-pad or left stick.
 * 
 * The LED blinks at a fixed position (47% offset) to create a persistence of vision
 * effect, indicating the robot's heading to the driver.
 * 
 * REQUIRES:
 * - Bluepad32 library: https://github.com/ricardoquesada/bluepad32-arduino
 * 
 * Features:
 * - Real-time radius adjustment for fine-tuning POV LED stability
 * - DShot600 protocol for ESC control
 * - Startup sequence: 2 seconds of 0 commands for ESC arming
 */

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_LIS331HH.h>
#include <Adafruit_Sensor.h>
#include <math.h>  // For M_PI and cos()
#include "led.h"  // LED handling module
#include "esc.h"  // ESC/PWM handling module
#include "radius_tuner.h"  // Radius tuning module

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================


// Accelerometer I2C pins
#define ACCEL_SDA 21
#define ACCEL_SCL 22

// ============================================================================
// MELTYBRAIN CONFIGURATION
// ============================================================================

#define MIN_TRACKING_RPM 400.0         // Minimum RPM for rotation tracking
#define MAX_TRACKING_RPM 3000.0        // Maximum RPM for LED window calculation
#define MELTYBRAIN_RPM_THRESHOLD 600.0 // RPM threshold to activate meltybrain mode
#define ROTATION_THRESHOLD 1.5          // Horizontal acceleration threshold (m/s²) to detect rotation
#define ROTATION_DEBOUNCE_MS 100        // Debounce time for rotation detection
#define TRANSLATION_TRIM 1.0            // Translation sensitivity multiplier
#define LED_OFFSET_PERCENT 20           // LED position in rotation (0-100%) for POV effect
#define MAX_TRANSLATION_ROTATION_INTERVAL_US ((1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_LIS331HH lis;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * Meltybrain control parameters for a single rotation
 * All timing values are calculated once per rotation to ensure consistency
 */
struct melty_parameters_t {
  float throttle_percent;              // Base throttle (0.0 to 1.0)
  unsigned long rotation_interval_us;  // Time for one complete rotation (microseconds)
  unsigned long led_start_us;          // LED on start time (microseconds into rotation)
  unsigned long led_stop_us;           // LED on stop time (microseconds into rotation)
  float target_direction_degrees;      // Target translation direction in degrees (0-360)
  float max_throttle_offset;           // Maximum throttle offset for translation modulation
};

// ============================================================================
// GLOBAL STATE
// ============================================================================

// Controller state
ControllerPtr myController = nullptr;
bool controllerConnected = false;

// Input state
uint8_t lastDpadState = 0;
float targetDirection = 0;  // Target direction in degrees (0-360)
float translationStrength = 0.0f;  // Translation strength from stick magnitude (0.0 to 1.0)
uint16_t currentThrottle = 0;  // Current throttle percentage (0-100)

// Accelerometer state
bool accelEnabled = false;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

// Rotation detection
bool isRotating = false;
unsigned long rotationDetectedTime = 0;
float current_rpm = 0;
float accel_correction_factor = 1.0;

// Accelerometer recalibration
bool recalibrateRequested = false;

// Config mode and radius tuning
bool configMode = false;
unsigned long lastCalibrationFlash = 0;
bool showCalibrationFlash = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void readAccelerometer();
void recalibrateAccelerometer();
float calculateRPM();
float getAccelForceG();
float getRotationIntervalMs();
struct melty_parameters_t getMeltyParameters();
void spin_one_rotation();
void updateHeadingLED(struct melty_parameters_t* params, unsigned long time_spent_us);
void updateMotorTranslation(struct melty_parameters_t* params, unsigned long time_spent_us);
void processController();

// ============================================================================
// BLUEPAD32 CALLBACKS
// ============================================================================

void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.printf("CALLBACK: Controller connected, idx=%d\n", ctl->index());
    myController = ctl;
    controllerConnected = true;
    
    // Disable scanning to prevent timeout crashes (Bluepad32 issue #43)
    BP32.enableNewBluetoothConnections(false);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.printf("CALLBACK: Controller disconnected, idx=%d\n", ctl->index());
    myController = nullptr;
    controllerConnected = false;
    leds.setColor(LED_COLOR_DISCONNECTED);
    setESCDShot(0);
    
    // Re-enable scanning for new controllers when disconnected
    BP32.enableNewBluetoothConnections(true);
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Mini Meltybrain Robot Controller");
  Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());
  
  // Initialize LEDs
  leds.begin();
  
  // Initialize ESC DShot (must be done before LED blink to ensure continuous arming signal)
  Serial.println("Initializing ESC DShot600...");
  escMotor.begin(DSHOT600);
  Serial.println("ESC DShot600 initialized - starting arming sequence...");
  
  // ESC arming: Send 0 commands for 2 seconds (delay acceptable during startup)
  for (int i = 0; i < ESC_STARTUP_MILLISECONDS; i++) {
    setESCDShot(0);  // Send 0 command
    delay(1);  // 1ms delay between commands
  }
  Serial.println("ESC arming sequence complete - ESC should be armed");
  
  // Initialize radius tuner
  radiusTuner.begin();
  
  // Startup blink sequence: red LEDs blink for 3 seconds
  Serial.println("Startup: Blinking red LEDs for 3 seconds...");
  unsigned long startupStart = millis();
  bool blinkState = false;
  while (millis() - startupStart < 3000) {
    blinkState = !blinkState;
    leds.setColor(blinkState ? LED_COLOR_DISCONNECTED : LED_COLOR_OFF);
    setESCDShot(0);  // Keep sending ESC signal during startup
    delay(250);
  }
  leds.setColor(LED_COLOR_DISCONNECTED);
  Serial.println("Startup complete - ESC should be armed");
  
  // Initialize accelerometer
  Wire.begin(ACCEL_SDA, ACCEL_SCL);
  bool accelInitialized = lis.begin_I2C();
  if (!accelInitialized) {
    // Try alternate I2C address
    accelInitialized = lis.begin_I2C(0x19);
  }
  
  if (accelInitialized) {
    Serial.println("LIS331HH Accelerometer found!");
    lis.setRange(LIS331HH_RANGE_24_G);
    lis.setDataRate(LIS331_DATARATE_1000_HZ);
    
  Serial.println("Calibrating accelerometer offsets (robot must be at rest)...");
    delay(100);  // Short pause for accelerometer warmup
    float sumX = 0, sumY = 0, sumZ = 0;
    const int CALIBRATION_SAMPLES = 60;
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
    accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;  // Remove gravity from Z
    Serial.print("Calibration complete - X offset: ");
    Serial.print(accelX_offset);
    Serial.print(", Y offset: ");
    Serial.print(accelY_offset);
    Serial.print(", Z offset: ");
    Serial.println(accelZ_offset);
    
    accelEnabled = true;
    Serial.println("LIS331HH initialized for meltybrain");
  } else {
    accelEnabled = false;
    Serial.println("LIS331HH not found - meltybrain features disabled");
  }
  
  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  
  Serial.println("Bluepad32 initialized - waiting for Stadia controller...");
  Serial.println("Put Stadia controller in pairing mode (hold Stadia button for 3 seconds)");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Update controller state
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processController();
  }
  
  // Handle accelerometer recalibration if requested
  if (recalibrateRequested && currentThrottle == 0) {
    recalibrateAccelerometer();
    recalibrateRequested = false;
  }
  
  // Handle calibration confirmation flash
  if (showCalibrationFlash) {
    unsigned long flashTime = millis() - lastCalibrationFlash;
    if (flashTime < 100) {
      leds.setColor(LED_COLOR_CALIBRATED);  // Yellow
    } else if (flashTime < 200) {
      leds.off();  // Off
    } else if (flashTime < 300) {
      leds.setColor(LED_COLOR_CALIBRATED);  // Yellow again
    } else if (flashTime < 400) {
      leds.off();  // Off
    } else {
      showCalibrationFlash = false;
      // Return to config mode color if in config mode
      if (configMode) {
        leds.setColor(LED_COLOR_CONFIG_MODE);
      }
    }
  }
  
  // Check if controller is connected
  bool disconnected = !controllerConnected || 
                      (myController == nullptr) || 
                      (myController != nullptr && !myController->isConnected());
  
  if (disconnected) {
    leds.setColor(LED_COLOR_DISCONNECTED);
    setESCDShot(0);  // Keep sending 0 commands to keep ESC armed
    return;
  }
  
  // If throttle is active, execute one rotation cycle
  if (currentThrottle > 0) {
    spin_one_rotation();
  } else {
    // Idle mode - keep ESC armed and update LED
    setESCDShot(0);  // Continuous 0 commands keep ESC armed
    static unsigned long lastLEDUpdate = 0;
    if (millis() - lastLEDUpdate > 100) {
      if (configMode && !showCalibrationFlash) {
        leds.setColor(LED_COLOR_CONFIG_MODE);
      } else if (!showCalibrationFlash) {
        leds.setColor(LED_COLOR_READY);
      }
      lastLEDUpdate = millis();
    }
  }
}

// ============================================================================
// CONTROLLER PROCESSING
// ============================================================================

/**
 * Process controller input: throttle, translation direction, and buttons
 */
void processController() {
  if (myController == nullptr || !myController->isConnected()) {
    return;
  }
  
  uint8_t dpad = myController->dpad();
  int16_t leftStickX = myController->axisX();
  int16_t leftStickY = myController->axisY();
  
  // Calculate stick magnitude and angle
  const int16_t LEFT_STICK_DEADZONE = 50;
  const float STICK_MAX = 512.0f;
  
  float stickX_norm = (float)leftStickX / STICK_MAX;
  float stickY_norm = (float)leftStickY / STICK_MAX;
  float stickMagnitude = sqrt(stickX_norm * stickX_norm + stickY_norm * stickY_norm);
  if (stickMagnitude > 1.0f) stickMagnitude = 1.0f;
  
  // Apply deadzone
  float deadzone_norm = (float)LEFT_STICK_DEADZONE / STICK_MAX;
  if (stickMagnitude < deadzone_norm) {
    stickMagnitude = 0.0f;
  } else {
    stickMagnitude = (stickMagnitude - deadzone_norm) / (1.0f - deadzone_norm);
  }
  
  // Calculate angle: 0° = forward (up), 90° = right, 180° = backward, 270° = left
  float stickAngle_rad = atan2(-stickY_norm, stickX_norm);
  float stickAngle_deg = stickAngle_rad * 180.0f / M_PI;
  if (stickAngle_deg < 0) stickAngle_deg += 360.0f;
  
  // D-pad takes priority, otherwise use left stick
  if (dpad != 0) {
    translationStrength = 1.0f;
    if (dpad == 1) targetDirection = 0;      // Up
    else if (dpad == 2) targetDirection = 90;   // Right
    else if (dpad == 4) targetDirection = 180;  // Down
    else if (dpad == 8) targetDirection = 270;  // Left
    lastDpadState = dpad;
  } else if (stickMagnitude > 0.0f) {
    targetDirection = stickAngle_deg;
    translationStrength = stickMagnitude;
    lastDpadState = 0;
  } else {
    translationStrength = 0.0f;
    lastDpadState = 0;
  }
  
  int16_t trigger = myController->brake();
  if (trigger == 0) trigger = myController->throttle();
  if (trigger == 0) {
    int16_t rightStickY = myController->axisRY();
    if (rightStickY > 0) trigger = rightStickY;
  }
  
  currentThrottle = (trigger > 0) ? map(trigger, 0, 512, 0, 100) : 0;
  if (currentThrottle > 100) currentThrottle = 100;
  
  // Config mode: Hold X for 2 seconds
  static bool xButtonHeld = false;
  static unsigned long xButtonHoldStart = 0;
  static bool xButtonToggled = false;
  if (myController->x()) {
    if (!xButtonHeld) {
      xButtonHeld = true;
      xButtonHoldStart = millis();
      xButtonToggled = false;
    } else if (!xButtonToggled && millis() - xButtonHoldStart >= 2000) {
      configMode = !configMode;
      xButtonToggled = true;
      leds.setColor(configMode ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    }
  } else {
    xButtonHeld = false;
    xButtonToggled = false;
  }
  
  // Radius adjustment: Right stick X (works while spinning)
  radiusTuner.processStickInput(myController->axisRX());
  
  // Config mode: Press A to confirm (optional)
  if (configMode) {
    static bool lastAButton = false;
    if (myController->a() && !lastAButton) {
      showCalibrationFlash = true;
      lastCalibrationFlash = millis();
    }
    lastAButton = myController->a();
  }
  
  // Recalibration: Hold Y for 1 second (only when stopped)
  static bool yButtonHeld = false;
  static unsigned long yButtonHoldStart = 0;
  if (currentThrottle == 0 && myController->y()) {
    if (!yButtonHeld) {
      yButtonHeld = true;
      yButtonHoldStart = millis();
    } else if (millis() - yButtonHoldStart >= 1000) {
      recalibrateRequested = true;
      yButtonHeld = false;
    }
  } else {
    yButtonHeld = false;
  }
}

// ============================================================================
// ACCELEROMETER FUNCTIONS
// ============================================================================

/**
 * Read accelerometer and apply calibration offsets
 * Updates lastAccelX, lastAccelY, lastAccelZ
 */
void readAccelerometer() {
  sensors_event_t event;
  if (lis.getEvent(&event)) {
    // Apply calibration offsets
    lastAccelX = event.acceleration.x - accelX_offset;
    lastAccelY = event.acceleration.y - accelY_offset;
    lastAccelZ = event.acceleration.z - accelZ_offset;
  }
}

/**
 * Recalibrate accelerometer (only when stopped)
 */
void recalibrateAccelerometer() {
  if (!accelEnabled) {
    Serial.println("Cannot recalibrate: accelerometer not enabled");
    return;
  }
  
  Serial.println("Recalibrating accelerometer offsets (robot must be at rest)...");
  
  delay(100);  // Wait for stability
  
  float sumX = 0, sumY = 0, sumZ = 0;
  const int CALIBRATION_SAMPLES = 60;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t event;
    if (lis.getEvent(&event)) {
      sumX += event.acceleration.x;
      sumY += event.acceleration.y;
      sumZ += event.acceleration.z;
    }
    
    delayMicroseconds(1000);  // 1ms delay between samples
  }
  
  accelX_offset = sumX / CALIBRATION_SAMPLES;
  accelY_offset = sumY / CALIBRATION_SAMPLES;
  accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;  // Z should be 1g when at rest
  
  Serial.print("Recalibration complete - X offset: ");
  Serial.print(accelX_offset);
  Serial.print(", Y offset: ");
  Serial.print(accelY_offset);
  Serial.print(", Z offset: ");
  Serial.println(accelZ_offset);
  
  // Reset rotation detection state
  isRotating = false;
  rotationDetectedTime = 0;
  current_rpm = 0;
}

/**
 * Get horizontal acceleration magnitude in G's
 */
float getAccelForceG() {
  readAccelerometer();
  return sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY) / 9.81;
}

/**
 * Calculate RPM from horizontal acceleration
 * Formula: a = ω²r → RPM = 60 * sqrt(a / r) / (2π)
 */
float calculateRPM() {
  float accel_g = fabs(getAccelForceG());
  
  if (accel_g <= 0 || isnan(accel_g) || isinf(accel_g)) {
    return 0.0;
  }
  
  float rpm = accel_g * 89445.0f / radiusTuner.getRadius();  // 89445 = 60 * 9.81 / (2π)²
  rpm = sqrt(rpm) * accel_correction_factor;
  
  if (isnan(rpm) || isinf(rpm) || rpm < 0) {
    return 0.0;
  }
  
  return rpm;
}

/**
 * Get rotation interval in milliseconds
 */
float getRotationIntervalMs() {
  float rpm = calculateRPM();
  if (rpm < MIN_TRACKING_RPM) rpm = MIN_TRACKING_RPM;
  return (1.0f / rpm) * 60 * 1000;
}

// ============================================================================
// MELTYBRAIN CONTROL FUNCTIONS
// ============================================================================

/**
 * Calculate meltybrain parameters for one rotation
 */
struct melty_parameters_t getMeltyParameters() {
  struct melty_parameters_t params = {};
  
  params.throttle_percent = currentThrottle / 100.0f;
  current_rpm = calculateRPM();
  
  float rotation_interval_ms = getRotationIntervalMs();
  params.rotation_interval_us = (unsigned long)(rotation_interval_ms * 1000);
  
  if (params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US) {
    params.rotation_interval_us = MAX_TRANSLATION_ROTATION_INTERVAL_US;
  }
  
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params.rotation_interval_us > 0 && 
                            params.throttle_percent > 0);
  
  params.target_direction_degrees = targetDirection;
  
  // Calculate modulation strength
  if (meltybrain_active && translationStrength > 0) {
    params.max_throttle_offset = (float)currentThrottle * translationStrength * TRANSLATION_TRIM;
    if (params.max_throttle_offset > currentThrottle * 0.5f) {
      params.max_throttle_offset = currentThrottle * 0.5f;
    }
  } else {
    params.max_throttle_offset = 0;
  }
  
  // Calculate LED POV timing (window size varies with RPM)
  float led_on_portion = current_rpm / MAX_TRACKING_RPM;
  if (led_on_portion < 0.10f) led_on_portion = 0.10f;
  if (led_on_portion > 0.90f) led_on_portion = 0.90f;
  
  unsigned long led_on_us = (unsigned long)(led_on_portion * params.rotation_interval_us);
  unsigned long led_offset_us = (unsigned long)(LED_OFFSET_PERCENT * params.rotation_interval_us / 100);
  
  long led_start = led_offset_us - (led_on_us / 2);
  if (led_start < 0) led_start += params.rotation_interval_us;
  
  long led_stop = led_start + led_on_us;
  if (led_stop >= (long)params.rotation_interval_us) led_stop -= params.rotation_interval_us;
  
  params.led_start_us = (unsigned long)led_start;
  params.led_stop_us = (unsigned long)led_stop;
  
  return params;
}

/**
 * Update LED for POV effect - blinks at fixed position in rotation
 */
void updateHeadingLED(struct melty_parameters_t* params, unsigned long time_spent_us) {
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params->rotation_interval_us > 0 && 
                            params->rotation_interval_us <= MAX_TRANSLATION_ROTATION_INTERVAL_US);
  
  if (!meltybrain_active) {
    leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    return;
  }
  
  // Check if in LED window for POV
  bool led_on;
  if (params->led_start_us > params->led_stop_us) {
    led_on = (time_spent_us >= params->led_start_us) || (time_spent_us <= params->led_stop_us);
  } else {
    led_on = (time_spent_us >= params->led_start_us && time_spent_us <= params->led_stop_us);
  }
  
  if (led_on) {
    leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_MELTYBRAIN);
  } else {
    leds.off();
  }
}

/**
 * Apply translation modulation: increase throttle when pointing toward target,
 * decrease when pointing away. Creates net force in target direction.
 */
void updateMotorTranslation(struct melty_parameters_t* params, unsigned long time_spent_us) {
  if (params->rotation_interval_us == 0 || params->throttle_percent == 0) {
    setESCDShot(0);
    return;
  }
  
  if (current_rpm < MELTYBRAIN_RPM_THRESHOLD || params->max_throttle_offset == 0) {
    setESCDShot(currentThrottle);
    return;
  }
  
  // Calculate phase difference
  float current_phase = (float)time_spent_us / (float)params->rotation_interval_us * 2.0f * M_PI;
  float target_phase = params->target_direction_degrees * M_PI / 180.0f;
  float phase_diff = current_phase - target_phase;
  
  // Normalize to -π to +π
  while (phase_diff > M_PI) phase_diff -= 2.0f * M_PI;
  while (phase_diff < -M_PI) phase_diff += 2.0f * M_PI;
  
  // Cosine modulation: aligned = +throttle, opposite = -throttle
  float throttle_offset = cos(phase_diff) * params->max_throttle_offset;
  float final_throttle = (currentThrottle / 100.0f) + (throttle_offset / 100.0f);
  
  if (final_throttle < 0) final_throttle = 0;
  if (final_throttle > 1.0) final_throttle = 1.0;
  setESCDShot((uint16_t)(final_throttle * 100));
}

/**
 * Execute one rotation cycle with meltybrain control
 * Blocking loop for precise timing - continuously sends DShot commands
 */
void spin_one_rotation() {
  if (!accelEnabled) {
    setESCDShot(currentThrottle);
    if (configMode && !showCalibrationFlash) {
      leds.setColor(LED_COLOR_CONFIG_MODE);
    } else {
      leds.setColor(LED_COLOR_READY);
    }
    return;
  }
  
  readAccelerometer();
  float horizontalAccel = sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY);
  
  // Detect rotation with debouncing
  unsigned long currentTime = millis();
  if (horizontalAccel > ROTATION_THRESHOLD) {
    if (!isRotating) {
      if (rotationDetectedTime == 0) {
        rotationDetectedTime = currentTime;
      } else if (currentTime - rotationDetectedTime >= ROTATION_DEBOUNCE_MS) {
        isRotating = true;
      }
    }
  } else {
    rotationDetectedTime = 0;
    isRotating = false;
  }
  
  struct melty_parameters_t melty_params = getMeltyParameters();
  
  // Exit early if not rotating or too slow
  if (!isRotating || melty_params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US || melty_params.rotation_interval_us == 0) {
    setESCDShot(currentThrottle);
    if (configMode && !showCalibrationFlash) {
      leds.setColor(LED_COLOR_CONFIG_MODE);
    } else if (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && melty_params.rotation_interval_us > 0) {
      leds.setColor(LED_COLOR_MELTYBRAIN);
    } else {
      leds.setColor(LED_COLOR_READY);
    }
    return;
  }
  
  if (melty_params.rotation_interval_us > 1000000) {
    setESCDShot(currentThrottle);
    leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    return;
  }
  
  // Blocking loop for one rotation
  unsigned long start_time = micros();
  unsigned long time_spent_this_rotation_us = 0;
  updateMotorTranslation(&melty_params, 0);
  
  // Alternate parameter updates to reduce timing errors
  static unsigned long cycle_count = 0;
  int parameter_update_offset_us = (++cycle_count % 2 == 1) ? melty_params.rotation_interval_us / 2 : 0;
  bool parameters_updated = false;
  
  while (time_spent_this_rotation_us < melty_params.rotation_interval_us) {
    // Check controller input
    static unsigned long last_controller_check = 0;
    if (micros() - last_controller_check > 200) {
      last_controller_check = micros();
      BP32.update();
      processController();
    }
    
    // Exit conditions
    if (currentThrottle == 0) {
      setESCDShot(0);
      leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
      break;
    }
    
    bool disconnected = !controllerConnected || 
                        (myController == nullptr) || 
                        (myController != nullptr && !myController->isConnected());
    if (disconnected) {
      setESCDShot(0);
      leds.setColor(LED_COLOR_DISCONNECTED);
      break;
    }
    
    // Update parameters once per rotation
    if (!parameters_updated && time_spent_this_rotation_us > parameter_update_offset_us) {
      melty_params = getMeltyParameters();
      parameters_updated = true;
      
      if (currentThrottle == 0 || melty_params.rotation_interval_us > 1000000 || melty_params.rotation_interval_us == 0) {
        setESCDShot(currentThrottle);
        break;
      }
    }
    
    updateMotorTranslation(&melty_params, time_spent_this_rotation_us);
    updateHeadingLED(&melty_params, time_spent_this_rotation_us);
    
    time_spent_this_rotation_us = micros() - start_time;
    if (time_spent_this_rotation_us > melty_params.rotation_interval_us * 2) {
      break;  // Safety: prevent infinite loop
    }
  }
}
