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
 *   while spinning using parabolic sine wave throttle modulation
 * 
 * LED Status Indicators:
 * - Red (solid): Controller disconnected
 * - Red (blinking): Startup sequence (3 seconds)
 * - Blue (solid): Connected and ready, or spinning up
 * - Green (POV): Meltybrain mode active, blinks at heading position (POV effect)
 * 
 * Meltybrain Operation:
 * The robot spins at maximum throttle when the trigger is held. Once RPM exceeds
 * the threshold, meltybrain mode activates. In this mode, the motor throttle is
 * modulated with a parabolic sine wave pattern synchronized to rotation. By applying the
 * modulation at specific phases of rotation, the robot translates in the direction
 * commanded by the D-pad or left stick.
 * 
 * The LED blinks at a fixed position (10% window) to create a persistence of vision
 * effect, indicating the robot's heading to the driver.
 * 
 * REQUIRES:
 * - Bluepad32 library: https://github.com/ricardoquesada/bluepad32-arduino
 * 
 * Features:
 * - Real-time radius adjustment for fine-tuning POV LED stability
 * - DShot600 protocol for ESC control
 * - Startup sequence: 2 seconds of 0 commands for ESC arming
 * - Resilient to rapid RPM changes during battle
 */

#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_LIS331HH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <Preferences.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define ACCEL_SDA 21
#define ACCEL_SCL 22
#define ESC_PIN 19
#define ESC_RMT_CHANNEL 1  // Avoid conflict with NeoPixels (which use RMT_CHANNEL_0)
#define NEOPIXEL_PIN 23
#define NUM_PIXELS 2
#define NEOPIXEL_BRIGHTNESS 50

// ============================================================================
// MELTYBRAIN CONFIGURATION
// ============================================================================

#define MIN_TRACKING_RPM 400.0
#define MAX_TRACKING_RPM 5000.0
#define MELTYBRAIN_RPM_THRESHOLD 600.0
#define ROTATION_THRESHOLD 1.5
#define ROTATION_DEBOUNCE_MS 100
#define TRANSLATION_TRIM_DEFAULT 1.0
#define LED_OFFSET_PERCENT 0
#define MAX_TRANSLATION_ROTATION_INTERVAL_US ((1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000)

// ============================================================================
// DSHOT PROTOCOL IMPLEMENTATION
// ============================================================================

// DShot constants
constexpr auto DSHOT_CLK_DIVIDER = 8;
constexpr auto DSHOT_PACKET_LENGTH = 17;
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21;
constexpr auto DSHOT_PAUSE_BIT = 16;

// DShot mode enumeration
typedef enum dshot_mode_e {
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// DShot packet structure
typedef struct dshot_packet_s {
    uint16_t throttle_value : 11;
    uint8_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// DShot configuration structure
typedef struct dshot_config_s {
    dshot_mode_t mode;
    bool is_bidirectional;
    gpio_num_t gpio_num;
    uint8_t pin_num;
    rmt_channel_t rmt_channel;
    uint8_t mem_block_num;
    uint16_t ticks_per_bit;
    uint8_t clk_div;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
} dshot_config_t;

// DShotRMT class for ESC control
class DShotRMT {
public:
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel) {
        dshot_config.gpio_num = gpio;
        dshot_config.pin_num = static_cast<uint8_t>(gpio);
        dshot_config.rmt_channel = rmtChannel;
        dshot_config.mem_block_num = static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel));
        buildTxRmtItem(DSHOT_NULL_PACKET);
    }

    ~DShotRMT() {
        rmt_driver_uninstall(dshot_config.rmt_channel);
    }

    bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false) {
        dshot_config.mode = dshot_mode;
        dshot_config.clk_div = DSHOT_CLK_DIVIDER;
        dshot_config.is_bidirectional = is_bidirectional;

        // Set timing parameters based on DShot mode
        switch (dshot_config.mode) {
            case DSHOT150:
                dshot_config.ticks_per_bit = 64;
                dshot_config.ticks_zero_high = 24;
                dshot_config.ticks_one_high = 48;
                break;
            case DSHOT300:
                dshot_config.ticks_per_bit = 32;
                dshot_config.ticks_zero_high = 12;
                dshot_config.ticks_one_high = 24;
                break;
            case DSHOT600:
                dshot_config.ticks_per_bit = 16;
                dshot_config.ticks_zero_high = 6;
                dshot_config.ticks_one_high = 12;
                break;
            case DSHOT1200:
                dshot_config.ticks_per_bit = 8;
                dshot_config.ticks_zero_high = 3;
                dshot_config.ticks_one_high = 6;
                break;
            default:
                dshot_config.ticks_per_bit = 0;
                dshot_config.ticks_zero_high = 0;
                dshot_config.ticks_one_high = 0;
                break;
        }

        dshot_config.ticks_zero_low = dshot_config.ticks_per_bit - dshot_config.ticks_zero_high;
        dshot_config.ticks_one_low = dshot_config.ticks_per_bit - dshot_config.ticks_one_high;

        dshot_tx_rmt_config.rmt_mode = RMT_MODE_TX;
        dshot_tx_rmt_config.channel = dshot_config.rmt_channel;
        dshot_tx_rmt_config.gpio_num = dshot_config.gpio_num;
        dshot_tx_rmt_config.mem_block_num = dshot_config.mem_block_num;
        dshot_tx_rmt_config.clk_div = dshot_config.clk_div;
        dshot_tx_rmt_config.tx_config.loop_en = false;
        dshot_tx_rmt_config.tx_config.carrier_en = false;
        dshot_tx_rmt_config.tx_config.idle_output_en = true;
        dshot_tx_rmt_config.tx_config.idle_level = is_bidirectional ? RMT_IDLE_LEVEL_HIGH : RMT_IDLE_LEVEL_LOW;

        rmt_config(&dshot_tx_rmt_config);
        return rmt_driver_install(dshot_tx_rmt_config.channel, 0, 0);
    }

    void sendThrottleValue(uint16_t throttle_value) {
        dshot_packet_t dshot_rmt_packet = {};
        if (throttle_value < DSHOT_THROTTLE_MIN && throttle_value > 0) {
            throttle_value = DSHOT_THROTTLE_MIN;
        }
        if (throttle_value > DSHOT_THROTTLE_MAX) {
            throttle_value = DSHOT_THROTTLE_MAX;
        }
        dshot_rmt_packet.throttle_value = throttle_value;
        dshot_rmt_packet.telemetric_request = 0;
        dshot_rmt_packet.checksum = calculateCRC(dshot_rmt_packet);
        sendRmtPaket(dshot_rmt_packet);
    }

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH];
    rmt_config_t dshot_tx_rmt_config;
    dshot_config_t dshot_config;

    rmt_item32_t *buildTxRmtItem(uint16_t parsed_packet) {
        if (dshot_config.is_bidirectional) {
            for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
                if (parsed_packet & 0b1000000000000000) {
                    dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_low;
                    dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_high;
                } else {
                    dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_low;
                    dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_high;
                }
                dshot_tx_rmt_item[i].level0 = 0;
                dshot_tx_rmt_item[i].level1 = 1;
            }
        } else {
            for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) {
                if (parsed_packet & 0b1000000000000000) {
                    dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
                    dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
                } else {
                    dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
                    dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
                }
                dshot_tx_rmt_item[i].level0 = 1;
                dshot_tx_rmt_item[i].level1 = 0;
            }
        }
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = dshot_config.is_bidirectional ? 1 : 0;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = dshot_config.is_bidirectional ? 0 : 1;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = DSHOT_PAUSE;
        return dshot_tx_rmt_item;
    }

    uint16_t calculateCRC(const dshot_packet_t &dshot_packet) {
        const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
        if (dshot_config.is_bidirectional) {
            const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
            return (~intermediate_result) & 0x0F;
        } else {
            return (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
        }
    }

    uint16_t parseRmtPaket(const dshot_packet_t &dshot_packet) {
        uint16_t crc = calculateCRC(dshot_packet);
        uint16_t parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
        parsedRmtPaket = (parsedRmtPaket << 4) | crc;
        return parsedRmtPaket;
    }

    void sendRmtPaket(const dshot_packet_t &dshot_packet) {
        buildTxRmtItem(parseRmtPaket(dshot_packet));
        rmt_write_items(dshot_tx_rmt_config.channel, dshot_tx_rmt_item, DSHOT_PACKET_LENGTH, false);
    }
};

// ============================================================================
// LED CONTROLLER
// ============================================================================

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
    LEDController() : pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800) {}

    void begin() {
  pixels.begin();
  pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
        off();
    }

    void setColor(LEDColor color) {
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

    void setColor(uint32_t color) {
        for (int i = 0; i < NUM_PIXELS; i++) {
            pixels.setPixelColor(i, color);
        }
        pixels.show();
    }

    void off() {
        setColor(0);
    }

private:
    Adafruit_NeoPixel pixels;
};

// ============================================================================
// RADIUS TUNER
// ============================================================================

class RadiusTuner {
public:
    RadiusTuner(float default_radius_cm)
        : current_radius_cm(default_radius_cm),
          default_radius_cm(default_radius_cm),
          min_radius_cm(0.5f),
          max_radius_cm(5.0f),
          radius_increment_cm(0.001f),
          lastStickX(0),
          lastRadiusUpdate(0),
          lastRadiusFlash(0) {}

    void begin() {
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

    bool processStickInput(int16_t stickX) {
        const int16_t STICK_DEADZONE = 100;
        const unsigned long UPDATE_INTERVAL_MS = 12;
        const unsigned long FLASH_INTERVAL_MS = 200;

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

    float getRadius() const { return current_radius_cm; }

    void setRadius(float radius_cm) {
        if (radius_cm < min_radius_cm) {
            current_radius_cm = min_radius_cm;
        } else if (radius_cm > max_radius_cm) {
            current_radius_cm = max_radius_cm;
  } else {
            current_radius_cm = radius_cm;
        }
    }

    void save() {
        Preferences preferences;
        preferences.begin("cruller", false);
        preferences.putFloat("radius_cm", current_radius_cm);
        preferences.end();
        Serial.printf("Saved radius to EEPROM: %.2f cm (%.1f mm)\n",
                      current_radius_cm, current_radius_cm * 10.0f);
    }

    void clear() {
        Preferences preferences;
        preferences.begin("cruller", false);
        preferences.remove("radius_cm");
        preferences.end();
        Serial.println("Cleared saved radius from EEPROM - will use default on next boot");
    }

private:
    float current_radius_cm;
    float default_radius_cm;
    float min_radius_cm;
    float max_radius_cm;
    float radius_increment_cm;
    int16_t lastStickX;
    unsigned long lastRadiusUpdate;
    unsigned long lastRadiusFlash;
};

// ============================================================================
// ESC CONTROL FUNCTIONS
// ============================================================================

DShotRMT escMotor((gpio_num_t)ESC_PIN, (rmt_channel_t)ESC_RMT_CHANNEL);

uint16_t throttleToDShot(int throttle) {
    if (throttle == 0) return 0;
    if (throttle > 0) return min(throttle, 999) + 48;
    return min(throttle * -1, 998) + 1049;
}

uint16_t throttlePercentToDShot(uint16_t throttlePercent) {
    if (throttlePercent > 100) throttlePercent = 100;
    if (throttlePercent == 0) return 0;
    int throttle_0_to_1000 = ((uint32_t)throttlePercent * 1000) / 100;
    if (throttle_0_to_1000 < 1) throttle_0_to_1000 = 1;
    if (throttle_0_to_1000 > 1000) throttle_0_to_1000 = 1000;
    return throttleToDShot(throttle_0_to_1000);
}

void initESC() {
    escMotor.begin(DSHOT600);
}

// ============================================================================
// GLOBAL OBJECTS AND STATE
// ============================================================================

Adafruit_LIS331HH lis;
LEDController leds;
RadiusTuner radiusTuner(2.4f);  // Default: 2.4cm (24mm)

// Controller state
ControllerPtr myController = nullptr;
bool controllerConnected = false;

// Input state
uint8_t lastDpadState = 0;
float targetDirection = 0;
float translationStrength = 0.0f;
uint16_t currentThrottle = 0;
float translationTrim = TRANSLATION_TRIM_DEFAULT;

// Accelerometer state
bool accelEnabled = false;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

// Rotation detection
bool isRotating = false;
unsigned long rotationDetectedTime = 0;
float current_rpm = 0;
float accel_correction_factor = 1.0;

// Rotation tracking
unsigned long rotation_started_at_us = 0;

// Config mode
bool configMode = false;
unsigned long lastCalibrationFlash = 0;
bool showCalibrationFlash = false;
bool recalibrateRequested = false;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct melty_parameters_t {
    float throttle_percent;
    unsigned long rotation_interval_us;
    unsigned long led_start_us;
    unsigned long led_stop_us;
    float target_direction_degrees;
    float max_throttle_offset;
};

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
        BP32.enableNewBluetoothConnections(false);
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.printf("CALLBACK: Controller disconnected, idx=%d\n", ctl->index());
        myController = nullptr;
        controllerConnected = false;
        leds.setColor(LED_COLOR_DISCONNECTED);
        escMotor.sendThrottleValue(0);
        BP32.enableNewBluetoothConnections(true);
    }
}

// ============================================================================
// ACCELEROMETER FUNCTIONS
// ============================================================================

void readAccelerometer() {
  sensors_event_t event;
  if (lis.getEvent(&event)) {
    lastAccelX = event.acceleration.x - accelX_offset;
    lastAccelY = event.acceleration.y - accelY_offset;
    lastAccelZ = event.acceleration.z - accelZ_offset;
  }
}

void recalibrateAccelerometer() {
  if (!accelEnabled) {
    Serial.println("Cannot recalibrate: accelerometer not enabled");
    return;
  }
  Serial.println("Recalibrating accelerometer offsets (robot must be at rest)...");
    delay(100);
  float sumX = 0, sumY = 0, sumZ = 0;
  const int CALIBRATION_SAMPLES = 60;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sensors_event_t event;
    if (lis.getEvent(&event)) {
      sumX += event.acceleration.x;
      sumY += event.acceleration.y;
      sumZ += event.acceleration.z;
    }
        delayMicroseconds(1000);
  }
  accelX_offset = sumX / CALIBRATION_SAMPLES;
  accelY_offset = sumY / CALIBRATION_SAMPLES;
    accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;
  Serial.print("Recalibration complete - X offset: ");
  Serial.print(accelX_offset);
  Serial.print(", Y offset: ");
  Serial.print(accelY_offset);
  Serial.print(", Z offset: ");
  Serial.println(accelZ_offset);
  isRotating = false;
  rotationDetectedTime = 0;
  current_rpm = 0;
}

float getAccelForceG() {
  readAccelerometer();
    return sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY) / 9.81;
}

float calculateRPM() {
  float accel_g = fabs(getAccelForceG());
    if (accel_g <= 0 || isnan(accel_g) || isinf(accel_g)) {
        return 0.0;
    }
    float radius_cm = radiusTuner.getRadius();
    float rpm = accel_g * 89445.0f / radius_cm;
    rpm = sqrt(rpm) * accel_correction_factor;
    if (isnan(rpm) || isinf(rpm) || rpm < 0) {
        return 0.0;
    }
  return rpm;
}

// Adaptive smoothing for rapid RPM changes during battle
static float smoothed_rotation_interval_ms = 0.0f;
static unsigned long last_interval_update = 0;
static const float SMOOTHING_FAST = 0.3f;
static const float SMOOTHING_SLOW = 0.1f;
static const float CHANGE_THRESHOLD = 0.15f;

float getRotationIntervalMs() {
  float rpm = calculateRPM();
    if (rpm < MIN_TRACKING_RPM) rpm = MIN_TRACKING_RPM;
    float new_interval_ms = (1.0f / rpm) * 60 * 1000;
    unsigned long now = millis();
    if (smoothed_rotation_interval_ms == 0.0f) {
        smoothed_rotation_interval_ms = new_interval_ms;
        last_interval_update = now;
    } else {
        float change_ratio = fabs(new_interval_ms - smoothed_rotation_interval_ms) / smoothed_rotation_interval_ms;
        float smoothing_factor = (change_ratio > CHANGE_THRESHOLD) ? SMOOTHING_FAST : SMOOTHING_SLOW;
        smoothed_rotation_interval_ms = smoothed_rotation_interval_ms * (1.0f - smoothing_factor) +
                                         new_interval_ms * smoothing_factor;
        last_interval_update = now;
    }
    return smoothed_rotation_interval_ms;
}

// ============================================================================
// MELTYBRAIN CONTROL FUNCTIONS
// ============================================================================

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
    if (meltybrain_active && translationStrength > 0) {
        params.max_throttle_offset = (float)currentThrottle * translationStrength * translationTrim;
        if (params.max_throttle_offset > currentThrottle * 0.5f) {
            params.max_throttle_offset = currentThrottle * 0.5f;
        }
  } else {
    params.max_throttle_offset = 0;
  }
  
    // Fixed LED on portion: 10% of rotation
    float led_on_portion = 0.10f;
  unsigned long led_on_us = (unsigned long)(led_on_portion * params.rotation_interval_us);
    float base_offset_percent = LED_OFFSET_PERCENT;
    float translation_offset_percent = 0.0f;
    if (params.max_throttle_offset > 0 && translationStrength > 0) {
        translation_offset_percent = params.target_direction_degrees / 360.0f * 100.0f;
    }
    float led_offset_percent = base_offset_percent + translation_offset_percent;
    if (led_offset_percent >= 100.0f) led_offset_percent -= 100.0f;
    unsigned long led_offset_us = (unsigned long)(led_offset_percent * params.rotation_interval_us / 100);
    long led_start = led_offset_us - (led_on_us / 2);
    if (led_start < 0) led_start += params.rotation_interval_us;
  long led_stop = led_start + led_on_us;
    if (led_stop >= (long)params.rotation_interval_us) led_stop -= params.rotation_interval_us;
  params.led_start_us = (unsigned long)led_start;
  params.led_stop_us = (unsigned long)led_stop;
  return params;
}

void updateHeadingLED(struct melty_parameters_t* params, unsigned long time_spent_us) {
  bool meltybrain_active = (current_rpm >= MELTYBRAIN_RPM_THRESHOLD && 
                            params->rotation_interval_us > 0 && 
                            params->rotation_interval_us <= MAX_TRANSLATION_ROTATION_INTERVAL_US);
  if (!meltybrain_active) {
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    return;
  }
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

void updateMotorTranslation(struct melty_parameters_t* params, unsigned long time_spent_us) {
  if (params->rotation_interval_us == 0 || params->throttle_percent == 0) {
        escMotor.sendThrottleValue(0);
    return;
  }
    if (current_rpm < MELTYBRAIN_RPM_THRESHOLD || params->max_throttle_offset == 0) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
    return;
  }
  
    // Parabolic sine wave approximation for throttle modulation
  long micros_into_phase = time_spent_us % (params->rotation_interval_us / 2);
    float phase_progress = 2.0f * micros_into_phase / (float)params->rotation_interval_us;
    float phase_offset_fraction = -4.0f * phase_progress * (phase_progress - 1.0f);
    float base_throttle_offset = phase_offset_fraction * params->max_throttle_offset;
    float current_phase = (float)time_spent_us / (float)params->rotation_interval_us * 2.0f * M_PI;
    while (current_phase >= 2.0f * M_PI) current_phase -= 2.0f * M_PI;
    while (current_phase < 0) current_phase += 2.0f * M_PI;
    float target_phase = params->target_direction_degrees * M_PI / 180.0f;
    float phase_diff = current_phase - target_phase;
    while (phase_diff > M_PI) phase_diff -= 2.0f * M_PI;
    while (phase_diff < -M_PI) phase_diff += 2.0f * M_PI;
    float throttle_offset;
    if (abs(phase_diff) < M_PI / 2.0f) {
        throttle_offset = base_throttle_offset;
  } else {
        throttle_offset = -base_throttle_offset;
  }
    float final_throttle = (currentThrottle / 100.0f) + (throttle_offset / 100.0f);
  if (final_throttle < 0) final_throttle = 0;
  if (final_throttle > 1.0) final_throttle = 1.0;
    escMotor.sendThrottleValue(throttlePercentToDShot((uint16_t)(final_throttle * 100)));
}

void spin_one_rotation() {
  if (!accelEnabled) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    return;
  }
  
  readAccelerometer();
  float horizontalAccel = sqrt(lastAccelX * lastAccelX + lastAccelY * lastAccelY);
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
  if (!isRotating || melty_params.rotation_interval_us > MAX_TRANSLATION_ROTATION_INTERVAL_US || melty_params.rotation_interval_us == 0) {
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
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
        escMotor.sendThrottleValue(throttlePercentToDShot(currentThrottle));
        leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
    return;
  }
  
    // Initialize rotation tracking
    if (rotation_started_at_us == 0) {
        rotation_started_at_us = micros();
    }

    long time_spent_this_rotation_us = micros() - rotation_started_at_us;
    static unsigned long last_rotation_interval_us = 0;
    if (last_rotation_interval_us > 0) {
        float interval_change_ratio = fabs((long)melty_params.rotation_interval_us - (long)last_rotation_interval_us) / (float)last_rotation_interval_us;
        if (interval_change_ratio > 0.2f) {
            rotation_started_at_us = micros();
            time_spent_this_rotation_us = 0;
        }
    }
    last_rotation_interval_us = melty_params.rotation_interval_us;

    if (time_spent_this_rotation_us > (long)melty_params.rotation_interval_us) {
        time_spent_this_rotation_us -= melty_params.rotation_interval_us;
        rotation_started_at_us += melty_params.rotation_interval_us;
    }

    updateMotorTranslation(&melty_params, time_spent_this_rotation_us);

    // Blocking loop for one rotation - resilient to RPM changes
    while (true) {
        time_spent_this_rotation_us = micros() - rotation_started_at_us;

        // Check for RPM changes every 10ms
        static unsigned long last_params_update = 0;
        if (micros() - last_params_update > 10000) {
            struct melty_parameters_t new_params = getMeltyParameters();
            float interval_change = fabs((long)new_params.rotation_interval_us - (long)melty_params.rotation_interval_us) / (float)melty_params.rotation_interval_us;
            if (interval_change > 0.15f) {
                melty_params = new_params;
                if (interval_change > 0.3f) {
                    rotation_started_at_us = micros();
                    time_spent_this_rotation_us = 0;
                }
                last_rotation_interval_us = melty_params.rotation_interval_us;
            }
            last_params_update = micros();
        }

        if (time_spent_this_rotation_us > (long)melty_params.rotation_interval_us) {
            time_spent_this_rotation_us -= melty_params.rotation_interval_us;
            rotation_started_at_us += melty_params.rotation_interval_us;
            melty_params = getMeltyParameters();
            last_rotation_interval_us = melty_params.rotation_interval_us;
        }

        updateMotorTranslation(&melty_params, time_spent_this_rotation_us);

    static unsigned long last_controller_check = 0;
        if (micros() - last_controller_check > 200) {
            last_controller_check = micros();
            BP32.update();
            processController();
        }

    if (currentThrottle == 0) {
            escMotor.sendThrottleValue(0);
            leds.setColor((configMode && !showCalibrationFlash) ? LED_COLOR_CONFIG_MODE : LED_COLOR_READY);
            break;
        }

    bool disconnected = !controllerConnected || 
                        (myController == nullptr) || 
                        (myController != nullptr && !myController->isConnected());
    if (disconnected) {
            escMotor.sendThrottleValue(0);
            leds.setColor(LED_COLOR_DISCONNECTED);
            break;
        }

        updateHeadingLED(&melty_params, time_spent_this_rotation_us);
        delayMicroseconds(10);
    }

      if (currentThrottle == 0) {
        rotation_started_at_us = 0;
    }
}

// ============================================================================
// CONTROLLER PROCESSING
// ============================================================================

void processController() {
    if (myController == nullptr || !myController->isConnected()) {
        return;
      }
      
    uint8_t dpad = myController->dpad();
    int16_t leftStickX = myController->axisX();
    int16_t leftStickY = myController->axisY();

    const int16_t LEFT_STICK_DEADZONE = 50;
    const float STICK_MAX = 512.0f;
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
    if (configMode && currentThrottle > 50) {
        currentThrottle = 50;
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
        } else if (!xButtonToggled && millis() - xButtonHoldStart >= 2000) {
            if (millis() - lastToggleTime >= 500) {
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
                    rotation_started_at_us = 0;
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
    const int16_t MODULATION_DEADZONE = 100;
    const unsigned long MODULATION_UPDATE_INTERVAL_MS = 100;
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
        } else if (millis() - yButtonHoldStart >= 1000) {
            recalibrateRequested = true;
            yButtonHeld = false;
        }
    } else {
        yButtonHeld = false;
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Mini Meltybrain Robot Controller");
    Serial.printf("Bluepad32 Firmware: %s\n", BP32.firmwareVersion());

    leds.begin();
    Serial.println("Initializing ESC DShot600...");
    initESC();
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

    Wire.begin(ACCEL_SDA, ACCEL_SCL);
    bool accelInitialized = lis.begin_I2C();
    if (!accelInitialized) {
        accelInitialized = lis.begin_I2C(0x19);
    }

    if (accelInitialized) {
        Serial.println("LIS331HH Accelerometer found!");
        lis.setRange(LIS331HH_RANGE_24_G);
        lis.setDataRate(LIS331_DATARATE_1000_HZ);
        Serial.println("Calibrating accelerometer offsets (robot must be at rest)...");
        delay(100);
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
        accelZ_offset = (sumZ / CALIBRATION_SAMPLES) - 9.81;
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
        processController();
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
        spin_one_rotation();
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
