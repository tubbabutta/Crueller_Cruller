/*
 * ESP32 Mini Meltybrain Robot Controller
 * 
 * Single-wheel combat robot controller with meltybrain translation control.
 * 
 * Hardware:
 * - Google Stadia controller (Bluetooth Classic via Bluepad32)
 * - Two daisy-chained NeoPixels on pin 23 (status indicators)
 * - AM32 ESC via PWM on pin 19 (motor control)
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
 * REQUIRES: Bluepad32 library
 * Install via: https://github.com/ricardoquesada/bluepad32-arduino
 */

#include <Bluepad32.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_LIS331HH.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// NeoPixel configuration
#define NEOPIXEL_PIN 23
#define NUM_PIXELS 2
#define NEOPIXEL_BRIGHTNESS 50

// ESC PWM configuration - matched to AM32 settings
#define ESC_PIN 19
#define PWM_FREQUENCY 400             // 400Hz PWM frequency (2.5ms period) for meltybrain control
                                       // Higher frequency allows smoother throttle modulation during rotation
#define PWM_RESOLUTION 16              // 16-bit resolution
#define PWM_CHANNEL 0
#define MIN_PULSE_WIDTH 1006           // Minimum pulse width (us) for ESC arming
#define MAX_PULSE_WIDTH 2006           // Maximum pulse width (us) for 100% throttle
#define PULSE_RANGE (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
#define PWM_PERIOD_US (1000000 / PWM_FREQUENCY)  // PWM period in microseconds

// Accelerometer I2C pins
#define ACCEL_SDA 21
#define ACCEL_SCL 22

// ============================================================================
// MELTYBRAIN CONFIGURATION
// ============================================================================

#define MIN_TRACKING_RPM 400.0         // Minimum RPM for rotation tracking
#define MAX_TRACKING_RPM 3000.0        // Maximum RPM for LED window calculation
#define MELTYBRAIN_RPM_THRESHOLD 600.0 // RPM threshold to activate meltybrain mode
#define ACCELEROMETER_HARDWARE_RADIUS_CM 3.4f  // Distance from center to accelerometer
#define ROTATION_THRESHOLD 1.5          // Horizontal acceleration threshold (m/s²) to detect rotation
#define ROTATION_DEBOUNCE_MS 100        // Debounce time for rotation detection
#define TRANSLATION_TRIM 1.0            // Translation sensitivity multiplier
#define LED_OFFSET_PERCENT 47           // Fixed LED offset percentage (0-100) for POV effect
#define MAX_TRANSLATION_ROTATION_INTERVAL_US ((1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
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
  int translate_forback;               // Translation direction: 1=forward, -1=backward, 0=neutral
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
float current_radius_cm = ACCELEROMETER_HARDWARE_RADIUS_CM;  // Runtime-adjustable radius
unsigned long lastCalibrationFlash = 0;
bool showCalibrationFlash = false;

// LED color constants (GRB order - Red and Green are swapped)
uint32_t COLOR_DISCONNECTED;
uint32_t COLOR_READY;
uint32_t COLOR_MELTYBRAIN;
uint32_t COLOR_CONFIG_MODE;
uint32_t COLOR_CALIBRATED;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void setPixelsColor(uint32_t color);
void setESCPWM(uint16_t throttlePercent);
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
    
    // Anguirel says: There is an issue where scan_evt will timeout eventually causing something to print "FEX x y",
    // (where x and y are various numbers) to the console and then eventually crash. Possible occurence of
    // https://github.com/ricardoquesada/bluepad32/issues/43.  The reported workaround is to disable scanning
    // for new controllers once a controller has connected.
    BP32.enableNewBluetoothConnections(false);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.printf("CALLBACK: Controller disconnected, idx=%d\n", ctl->index());
    myController = nullptr;
    controllerConnected = false;
    setPixelsColor(COLOR_DISCONNECTED);
    setESCPWM(0);
    
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
  
  // Initialize NeoPixels
  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
  
  // Initialize color constants (GRB order - swap R and G for correct colors)
  COLOR_DISCONNECTED = pixels.Color(0, 255, 0);  // Red
  COLOR_READY = pixels.Color(0, 0, 255);           // Blue
  COLOR_MELTYBRAIN = pixels.Color(255, 0, 0);     // Green
  COLOR_CONFIG_MODE = pixels.Color(255, 0, 255);   // Purple (GRB: full G, no R, full B)
  COLOR_CALIBRATED = pixels.Color(255, 255, 0);   // Yellow (GRB: full G, full R, no B)
  
  // Initialize ESC PWM (must be done before LED blink to ensure continuous arming signal)
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ESC_PIN, PWM_CHANNEL);
  Serial.println("Initializing ESC - sending minimum pulse (1006us) for arming...");
  setESCPWM(0);
  
  // Startup blink sequence: red LEDs blink for 3 seconds
  Serial.println("Startup: Blinking red LEDs for 3 seconds...");
  unsigned long startupStart = millis();
  bool blinkState = false;
  while (millis() - startupStart < 3000) {
    blinkState = !blinkState;
    setPixelsColor(blinkState ? COLOR_DISCONNECTED : 0);
    setESCPWM(0);  // Keep sending ESC signal during startup
    delay(250);
  }
  setPixelsColor(COLOR_DISCONNECTED);
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
    
    // Calibrate accelerometer offsets
    // Sample accelerometer while robot is at rest to determine zero offsets
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
    accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;  // Z should be 1g when at rest
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
      setPixelsColor(COLOR_CALIBRATED);  // Yellow
    } else if (flashTime < 200) {
      setPixelsColor(0);  // Off
    } else if (flashTime < 300) {
      setPixelsColor(COLOR_CALIBRATED);  // Yellow again
    } else if (flashTime < 400) {
      setPixelsColor(0);  // Off
    } else {
      showCalibrationFlash = false;
      // Return to config mode color if in config mode
      if (configMode) {
        setPixelsColor(COLOR_CONFIG_MODE);
      }
    }
  }
  
  // Check if controller is connected
  bool disconnected = !controllerConnected || 
                      (myController == nullptr) || 
                      (myController != nullptr && !myController->isConnected());
  
  if (disconnected) {
    setPixelsColor(COLOR_DISCONNECTED);
    setESCPWM(0);
    delay(50);
    return;
  }
  
  // If throttle is active, execute one rotation cycle
  if (currentThrottle > 0) {
    spin_one_rotation();
  } else {
    // Idle mode - show ready LED or config mode color
    if (configMode && !showCalibrationFlash) {
      setPixelsColor(COLOR_CONFIG_MODE);
    } else if (!showCalibrationFlash) {
      setPixelsColor(COLOR_READY);
    }
    setESCPWM(0);
    delay(50);
  }
}

// ============================================================================
// CONTROLLER PROCESSING
// ============================================================================

/**
 * Process controller input and update global state
 * Reads D-pad, left stick, and right trigger
 */
void processController() {
  if (myController == nullptr || !myController->isConnected()) {
    return;
  }
  
  // Get D-pad and left stick values
  uint8_t dpad = myController->dpad();
  int16_t leftStickX = myController->axisX();
  int16_t leftStickY = myController->axisY();
  
  // Determine direction from left stick (with deadzone)
  const int16_t STICK_DEADZONE = 50;
  uint8_t stickDirection = 0;
  
  if (abs(leftStickX) < STICK_DEADZONE && abs(leftStickY) < STICK_DEADZONE) {
    stickDirection = 0;  // No stick input
  } else {
    // Determine primary direction from stick
    int16_t absX = abs(leftStickX);
    int16_t absY = abs(leftStickY);
    if (absY > absX) {
      stickDirection = (leftStickY < 0) ? 1 : 4;  // Up or Down
    } else {
      stickDirection = (leftStickX > 0) ? 2 : 8;   // Right or Left
    }
  }
  
  // Use D-pad if pressed, otherwise use left stick
  uint8_t activeDirection = (dpad != 0) ? dpad : stickDirection;
  
  // Map direction to target angle (0-360 degrees)
  if (activeDirection == 1) {
    targetDirection = 0;    // Up = Forward (0°)
  } else if (activeDirection == 2) {
    targetDirection = 90;   // Right (90°)
  } else if (activeDirection == 4) {
    targetDirection = 180;  // Down = Backward (180°)
  } else if (activeDirection == 8) {
    targetDirection = 270;  // Left (270°)
  }
  // If activeDirection == 0, keep previous targetDirection
  
  lastDpadState = activeDirection;
  
  // Get throttle from right trigger
  // Try multiple methods as controller APIs may vary
  int16_t trigger = myController->brake();
  if (trigger == 0) {
    trigger = myController->throttle();
  }
  if (trigger == 0) {
    int16_t rightStickY = myController->axisRY();
    if (rightStickY > 0) {
      trigger = rightStickY;
    }
  }
  
  // Map trigger value (0-512) to throttle percentage (0-100%)
  currentThrottle = (trigger > 0) ? map(trigger, 0, 512, 0, 100) : 0;
  if (currentThrottle > 100) currentThrottle = 100;
  
  // Config mode toggle: Hold X button for 2 seconds
  static bool xButtonHeld = false;
  static unsigned long xButtonHoldStart = 0;
  const unsigned long CONFIG_MODE_HOLD_MS = 2000;
  
  if (myController->x()) {
    if (!xButtonHeld) {
      xButtonHeld = true;
      xButtonHoldStart = millis();
    } else if (millis() - xButtonHoldStart >= CONFIG_MODE_HOLD_MS) {
      configMode = !configMode;
      xButtonHeld = false;
      if (configMode) {
        // Enter config mode - show purple
        setPixelsColor(COLOR_CONFIG_MODE);
      } else {
        // Exit config mode - return to normal
        setPixelsColor(COLOR_READY);
      }
    }
  } else {
    xButtonHeld = false;
  }
  
  // In config mode, use right stick X to adjust radius
  if (configMode) {
    int16_t rightStickX = myController->axisRX();
    float stickValue = (float)rightStickX / 512.0f;  // -1.0 to +1.0
    
    const float MIN_RADIUS_CM = 1.0f;   // 10mm
    const float MAX_RADIUS_CM = 6.0f;   // 60mm
    float new_radius = MIN_RADIUS_CM + (stickValue + 1.0f) * 0.5f * (MAX_RADIUS_CM - MIN_RADIUS_CM);
    
    if (new_radius < MIN_RADIUS_CM) new_radius = MIN_RADIUS_CM;
    if (new_radius > MAX_RADIUS_CM) new_radius = MAX_RADIUS_CM;
    
    if (abs(new_radius - current_radius_cm) > 0.05f) {  // Update if changed significantly
      current_radius_cm = new_radius;
    }
    
    // Save calibration point: Press A button
    static bool lastAButton = false;
    if (myController->a() && !lastAButton) {
      // Trigger calibration flash
      showCalibrationFlash = true;
      lastCalibrationFlash = millis();
      // TODO: Save calibration point to lookup table (if implementing Option 2)
    }
    lastAButton = myController->a();
  }
  
  // Handle accelerometer recalibration (only when throttle is zero)
  // Hold Y button for 1 second to trigger recalibration
  static bool yButtonHeld = false;
  static unsigned long yButtonHoldStart = 0;
  static bool recalibrationTriggered = false;
  const unsigned long RECALIBRATE_HOLD_MS = 1000;  // Hold for 1 second
  
  if (currentThrottle == 0) {  // Only allow recalibration when stopped
    if (myController->y()) {
      if (!yButtonHeld) {
        yButtonHeld = true;
        yButtonHoldStart = millis();
        recalibrationTriggered = false;
      } else {
        // Check if held long enough
        if (!recalibrationTriggered && (millis() - yButtonHoldStart >= RECALIBRATE_HOLD_MS)) {
          recalibrationTriggered = true;
          recalibrateRequested = true;
          Serial.println("Recalibration triggered - Y button held for 1 second");
        }
      }
    } else {
      // Button released - reset state
      yButtonHeld = false;
      recalibrationTriggered = false;
    }
  } else {
    // Reset if throttle becomes active
    yButtonHeld = false;
    recalibrationTriggered = false;
    recalibrateRequested = false;
  }
}

// ============================================================================
// HARDWARE CONTROL FUNCTIONS
// ============================================================================

/**
 * Set all NeoPixels to the specified color
 * @param color 32-bit color value (GRB format)
 */
void setPixelsColor(uint32_t color) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

/**
 * Set ESC PWM output based on throttle percentage
 * @param throttlePercent Throttle percentage (0-100)
 */
void setESCPWM(uint16_t throttlePercent) {
  if (throttlePercent > 100) throttlePercent = 100;
  
  // Calculate pulse width in microseconds
  uint32_t pulseWidthUs = MIN_PULSE_WIDTH + ((uint32_t)throttlePercent * PULSE_RANGE) / 100;
  
  // Convert to PWM duty cycle
  // Note: Higher PWM frequency (400Hz) allows much faster throttle updates for meltybrain control
  // At 400Hz, we get 400 updates per second vs 50 at 50Hz, enabling smooth modulation during rotation
  uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;
  uint32_t duty = (pulseWidthUs * maxDuty) / PWM_PERIOD_US;
  ledcWrite(PWM_CHANNEL, duty);
}

// ============================================================================
// ACCELEROMETER FUNCTIONS
// ============================================================================

/**
 * Read accelerometer and apply calibration offsets
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
 * Recalibrate accelerometer offsets
 * Should only be called when robot is at rest (throttle = 0)
 * Samples accelerometer readings to determine zero offsets
 */
void recalibrateAccelerometer() {
  if (!accelEnabled) {
    Serial.println("Cannot recalibrate: accelerometer not enabled");
    return;
  }
  
  Serial.println("Recalibrating accelerometer offsets (robot must be at rest)...");
  
  // Sample accelerometer while robot is at rest
  delay(100);  // Short pause
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
 * @return Horizontal acceleration in G's
 */
float getAccelForceG() {
  readAccelerometer();
  // Calculate horizontal acceleration magnitude (X and Y components)
  float horizontalAccelG = sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY) / 9.81;
  return horizontalAccelG;
}

/**
 * Calculate RPM from accelerometer data
 * Uses centripetal acceleration formula: a = ω²r = (2πf)²r
 * Solving for RPM: RPM = 60 * sqrt(a / r) / (2π)
 * @return Calculated RPM
 */
float calculateRPM() {
  float accel_g = fabs(getAccelForceG());
  float rpm = accel_g * 89445.0f;  // 89445 = 60 * 9.81 / (2π)²
  rpm = rpm / current_radius_cm;  // Use runtime-adjustable radius
  rpm = sqrt(rpm);
  rpm *= accel_correction_factor;  // Apply correction factor if needed
  return rpm;
}

/**
 * Get rotation interval in milliseconds
 * @return Rotation interval in milliseconds
 */
float getRotationIntervalMs() {
  float rpm = calculateRPM();
  
  // Enforce minimum RPM to prevent division by zero
  if (rpm < MIN_TRACKING_RPM) {
    rpm = MIN_TRACKING_RPM;
  }
  
  float rotation_interval_ms = (1.0f / rpm) * 60 * 1000;
  return rotation_interval_ms;
}

// ============================================================================
// MELTYBRAIN CONTROL FUNCTIONS
// ============================================================================

/**
 * Calculate all meltybrain parameters for one rotation
 * Parameters are calculated once per rotation to ensure timing consistency
 * @return melty_parameters_t structure with all calculated values
 */
struct melty_parameters_t getMeltyParameters() {
  struct melty_parameters_t params = {};
  
  // Base throttle percentage (0.0 to 1.0)
  params.throttle_percent = currentThrottle / 100.0f;
  
  // Calculate rotation interval from current RPM
  float rotation_interval_ms = getRotationIntervalMs();
  params.rotation_interval_us = (unsigned long)(rotation_interval_ms * 1000);
  
  // Limit rotation interval for safety (prevent extremely slow rotations)
  if (params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US) {
    params.rotation_interval_us = MAX_TRANSLATION_ROTATION_INTERVAL_US;
  }
  
  // Calculate current RPM
  current_rpm = calculateRPM();
  
  // Determine if meltybrain mode should be active
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params.rotation_interval_us > 0 && 
                            params.throttle_percent > 0);
  
  // Determine translation direction from controller input
  if (lastDpadState != 0) {
    // Map target direction to forward/backward/neutral
    if (targetDirection == 0 || targetDirection == 90) {
      params.translate_forback = 1;  // Forward
    } else if (targetDirection == 180 || targetDirection == 270) {
      params.translate_forback = -1; // Backward
    } else {
      params.translate_forback = 0;  // Neutral
    }
  } else {
    params.translate_forback = 0;  // No input = neutral
  }
  
  // Calculate maximum throttle offset for translation modulation
  // Formula: max_throttle_offset = translate_forback * throttle_perk * translate_trim / 1024
  // This scales the modulation based on current throttle and translation direction
  if (meltybrain_active && params.translate_forback != 0) {
    int translate_value = params.translate_forback * 512;  // Scale to -512 to +512 range
    int throttle_perk_scaled = currentThrottle * 1023 / 100;  // Scale 0-100 to 0-1023
    params.max_throttle_offset = (float)(translate_value * throttle_perk_scaled * TRANSLATION_TRIM) / 1024.0f;
    params.max_throttle_offset = params.max_throttle_offset * 100.0f / 1023.0f;  // Scale back to 0-100 range
  } else {
    params.max_throttle_offset = 0;
  }
  
  // Calculate LED timing for persistence of vision (POV) effect
  // LED window size varies with RPM: faster rotation = smaller window
  float led_on_portion = current_rpm / MAX_TRACKING_RPM;
  if (led_on_portion < 0.10f) led_on_portion = 0.10f;  // Clamp to 10% minimum
  if (led_on_portion > 0.90f) led_on_portion = 0.90f;  // Clamp to 90% maximum
  
  // Calculate LED window size in microseconds
  unsigned long led_on_us = (unsigned long)(led_on_portion * params.rotation_interval_us);
  
  // Calculate LED offset (fixed position in rotation)
  unsigned long led_offset_us = (unsigned long)(LED_OFFSET_PERCENT * params.rotation_interval_us / 100);
  
  // Calculate LED start/stop times centered on the fixed offset
  long led_start = led_offset_us - (led_on_us / 2);
  if (led_start < 0) {
    led_start += params.rotation_interval_us;  // Wrap around if negative
  }
  
  long led_stop = led_start + led_on_us;
  if (led_stop >= (long)params.rotation_interval_us) {
    led_stop -= params.rotation_interval_us;  // Wrap around if exceeds rotation
  }
  
  params.led_start_us = (unsigned long)led_start;
  params.led_stop_us = (unsigned long)led_stop;
  
  return params;
}

/**
 * Update heading LED based on rotation timing
 * Creates persistence of vision (POV) effect by blinking LED at fixed position
 * @param params Meltybrain parameters for current rotation
 * @param time_spent_us Time spent in current rotation (microseconds)
 */
void updateHeadingLED(struct melty_parameters_t* params, unsigned long time_spent_us) {
  // In config mode, show config color instead of meltybrain colors
  if (configMode && !showCalibrationFlash) {
    setPixelsColor(COLOR_CONFIG_MODE);
    return;
  }
  
  // Check if meltybrain mode is active
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params->rotation_interval_us > 0 && 
                            params->rotation_interval_us <= MAX_TRANSLATION_ROTATION_INTERVAL_US);
  
  if (!meltybrain_active) {
    // Not in meltybrain mode - show ready (blue)
    setPixelsColor(COLOR_READY);
    return;
  }
  
  // Meltybrain mode is active - use POV effect
  // Check if we're in the LED window (blink at fixed position)
  bool led_on = false;
  if (params->led_start_us > params->led_stop_us) {
    // Window wraps around rotation boundary (e.g., start=350°, stop=10°)
    led_on = (time_spent_us >= params->led_start_us) || (time_spent_us <= params->led_stop_us);
  } else {
    // Normal case (e.g., start=10°, stop=20°)
    led_on = (time_spent_us >= params->led_start_us && time_spent_us <= params->led_stop_us);
  }
  
  if (led_on) {
    setPixelsColor(COLOR_MELTYBRAIN);  // Green when in LED window (POV effect)
  } else {
    setPixelsColor(0);  // Off for POV effect
  }
}

/**
 * Update motor throttle with translation modulation
 * Applies sine wave modulation synchronized to rotation for directional translation
 * @param params Meltybrain parameters for current rotation
 * @param time_spent_us Time spent in current rotation (microseconds)
 */
void updateMotorTranslation(struct melty_parameters_t* params, unsigned long time_spent_us) {
  if (params->rotation_interval_us == 0 || params->throttle_percent == 0) {
    setESCPWM(0);
    return;
  }
  
  // Check if meltybrain mode is active
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params->max_throttle_offset != 0);
  
  if (!meltybrain_active) {
    // Normal mode - just apply base throttle
    setESCPWM(currentThrottle);
    return;
  }
  
  // Meltybrain mode - apply sine wave modulation for translation
  // Calculate phase progress over HALF rotation (sine wave period)
  long micros_into_phase = time_spent_us % (params->rotation_interval_us / 2);
  float phase_progress = 2.0 * micros_into_phase / params->rotation_interval_us;
  
  // Parabolic approximation of sine wave
  // Goes from Y=0 to 1 and back to 0 in the range X=0..1
  // This approximates sin(πx) using a parabola: -4x(x-1)
  float phase_offset_fraction = -4 * phase_progress * (phase_progress - 1);
  
  // Calculate throttle offset from sine wave
  double throttle_offset = (double)(phase_offset_fraction * params->max_throttle_offset);
  
  // Apply offset based on rotation phase
  // The rotation is divided into two phases (0°-180° and 180°-360°)
  // For forward translation: apply +offset in first phase, -offset in second phase
  // For backward translation: apply -offset in first phase, +offset in second phase
  // This creates the directional thrust needed for translation
  bool in_phase_1 = (time_spent_us >= 0 && time_spent_us < params->rotation_interval_us / 2);
  
  float final_throttle;
  if (params->translate_forback > 0) {
    // Forward translation
    final_throttle = (currentThrottle / 100.0f) + (in_phase_1 ? throttle_offset : -throttle_offset) / 100.0f;
  } else if (params->translate_forback < 0) {
    // Backward translation (swapped phases)
    final_throttle = (currentThrottle / 100.0f) + (in_phase_1 ? -throttle_offset : throttle_offset) / 100.0f;
  } else {
    // No translation - just base throttle
    final_throttle = currentThrottle / 100.0f;
  }
  
  // Clamp to valid range (0.0 to 1.0)
  if (final_throttle < 0) final_throttle = 0;
  if (final_throttle > 1.0) final_throttle = 1.0;
  
  // Convert to percentage and apply
  uint16_t throttle_percent = (uint16_t)(final_throttle * 100);
  setESCPWM(throttle_percent);
}

/**
 * Execute one complete rotation cycle with meltybrain control
 * Uses a blocking loop to ensure precise timing for motor modulation and LED updates
 * Continuously checks for controller input and exit conditions
 */
void spin_one_rotation() {
  if (!accelEnabled) {
    // No accelerometer - just apply throttle directly
    setESCPWM(currentThrottle);
    if (configMode && !showCalibrationFlash) {
      setPixelsColor(COLOR_CONFIG_MODE);
    } else {
      setPixelsColor(COLOR_READY);
    }
    delay(10);
    return;
  }
  
  // Read accelerometer to check for rotation
  readAccelerometer();
  float horizontalAccel = sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY);
  
  unsigned long currentTime = millis();
  
  // Detect rotation with debouncing
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
  
  // Get meltybrain parameters (calculated once per rotation)
  struct melty_parameters_t melty_params = getMeltyParameters();
  
  // Debug output (occasionally to avoid flooding serial)
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 500) {
    last_debug = millis();
    Serial.print("Rotating: ");
    Serial.print(isRotating ? "YES" : "NO");
    Serial.print(" | RPM: ");
    Serial.print(current_rpm);
    Serial.print(" | Threshold: ");
    Serial.print(MELTYBRAIN_RPM_THRESHOLD);
    Serial.print(" | Meltybrain: ");
    Serial.print((current_rpm >= MELTYBRAIN_RPM_THRESHOLD) ? "ACTIVE" : "INACTIVE");
    Serial.print(" | Rotation interval: ");
    Serial.print(melty_params.rotation_interval_us);
    Serial.println(" us");
  }
  
  // If not rotating or too slow, just apply throttle directly (non-blocking)
  if (!isRotating || melty_params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US || melty_params.rotation_interval_us == 0) {
    setESCPWM(currentThrottle);
    // Show LED based on mode
    if (configMode && !showCalibrationFlash) {
      setPixelsColor(COLOR_CONFIG_MODE);
    } else if (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && melty_params.rotation_interval_us > 0) {
      setPixelsColor(COLOR_MELTYBRAIN);
    } else {
      setPixelsColor(COLOR_READY);
    }
    delay(10);
    return;
  }
  
  // Safety: Limit maximum rotation interval to prevent infinite loops
  if (melty_params.rotation_interval_us > 1000000) {  // 1 second max
    setESCPWM(currentThrottle);
    if (configMode && !showCalibrationFlash) {
      setPixelsColor(COLOR_CONFIG_MODE);
    } else {
      setPixelsColor(COLOR_READY);
    }
    delay(10);
    return;
  }
  
  // Capture start time for this rotation
  unsigned long start_time = micros();
  unsigned long time_spent_this_rotation_us = 0;
  
  // Track cycle count for alternating parameter updates
  // Updating parameters at different phases helps cancel timing errors
  static unsigned long cycle_count = 0;
  cycle_count++;
  
  // Update parameters at beginning or midpoint (alternating) to cancel timing errors
  int parameter_update_offset_us = 0;
  if (cycle_count % 2 == 1) {
    parameter_update_offset_us = melty_params.rotation_interval_us / 2;
  }
  bool parameters_updated = false;
  
  // Blocking loop for one complete rotation
  // This ensures precise timing for motor modulation and LED updates
  while (time_spent_this_rotation_us < melty_params.rotation_interval_us) {
    
    // Check for controller updates frequently (every ~200us) for maximum responsiveness
    static unsigned long last_controller_check = 0;
    unsigned long current_micros = micros();
    if (current_micros - last_controller_check > 200) {
      last_controller_check = current_micros;
      BP32.update();  // Always call update, even if no data
      processController();  // Always process to update currentThrottle
    }
    
    // EXIT CONDITION 1: Check if throttle is now zero (user released trigger)
    if (currentThrottle == 0) {
      setESCPWM(0);
      if (configMode && !showCalibrationFlash) {
        setPixelsColor(COLOR_CONFIG_MODE);
      } else {
        setPixelsColor(COLOR_READY);
      }
      return;
    }
    
    // EXIT CONDITION 2: Check if controller disconnected
    bool disconnected = !controllerConnected || 
                        (myController == nullptr) || 
                        (myController != nullptr && !myController->isConnected());
    if (disconnected) {
      setESCPWM(0);
      setPixelsColor(COLOR_DISCONNECTED);
      return;
    }
    
    // Update meltybrain parameters if update time has elapsed
    // Parameters are updated once per rotation to maintain consistency
    if (!parameters_updated && time_spent_this_rotation_us > parameter_update_offset_us) {
      melty_params = getMeltyParameters();
      parameters_updated = true;
      
      // Re-check exit conditions after parameter update
      if (currentThrottle == 0) {
        setESCPWM(0);
        if (configMode && !showCalibrationFlash) {
          setPixelsColor(COLOR_CONFIG_MODE);
        } else {
          setPixelsColor(COLOR_READY);
        }
        return;
      }
      
      // Safety: Re-check rotation interval
      if (melty_params.rotation_interval_us > 1000000 || melty_params.rotation_interval_us == 0) {
        setESCPWM(currentThrottle);
        if (configMode && !showCalibrationFlash) {
          setPixelsColor(COLOR_CONFIG_MODE);
        } else {
          setPixelsColor(COLOR_READY);
        }
        return;
      }
    }
    
    // Update motor with translation modulation
    updateMotorTranslation(&melty_params, time_spent_this_rotation_us);
    
    // Update heading LED (POV effect)
    updateHeadingLED(&melty_params, time_spent_this_rotation_us);
    
    // Update time spent in rotation
    time_spent_this_rotation_us = micros() - start_time;
    
    // Safety: Prevent infinite loop if timing goes wrong
    if (time_spent_this_rotation_us > melty_params.rotation_interval_us * 2) {
      // Something went wrong with timing - exit
      break;
    }
  }
}
