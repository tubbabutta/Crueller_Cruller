// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
// Pin assignments and hardware interface settings

// I2C pins for accelerometer communication
#define ACCEL_SDA 21  // I2C data line for accelerometer
#define ACCEL_SCL 22  // I2C clock line for accelerometer

// ESC (Electronic Speed Controller) pin configuration
#define ESC_PIN 19                    // GPIO pin connected to ESC signal line
#define ESC_RMT_CHANNEL 1             // RMT channel for DShot protocol (avoid conflict with NeoPixels on channel 0)

// NeoPixel LED configuration
#define NEOPIXEL_PIN 23               // GPIO pin connected to NeoPixel data line
#define NUM_PIXELS 2                  // Number of NeoPixel LEDs in the strip
#define NEOPIXEL_BRIGHTNESS 50        // LED brightness (0-255), lower values reduce power consumption

// ============================================================================
// MELTYBRAIN CONFIGURATION
// ============================================================================
// Parameters that control meltybrain translation behavior

// RPM tracking limits
#define MIN_TRACKING_RPM 400.0        // Minimum RPM for reliable rotation tracking (below this, translation disabled)
#define MAX_TRACKING_RPM 5000.0       // Maximum RPM for tracking calculations (safety limit)

// Meltybrain activation threshold
#define MELTYBRAIN_RPM_THRESHOLD 600.0  // RPM required to activate meltybrain translation (prevents false activation)

// Rotation detection parameters
#define ROTATION_THRESHOLD 1.5        // Horizontal acceleration (g-force) threshold to detect rotation
#define ROTATION_DEBOUNCE_MS 100      // Time (ms) to wait before confirming rotation detection (prevents false triggers)

// Translation control
#define TRANSLATION_TRIM_DEFAULT 1.0  // Default modulation strength multiplier (1.0 = 100%, range: 0.0-2.0)
#define LED_OFFSET_PERCENT 0          // Base LED position offset (0-100%, adjusts where LED appears at rest)

// Maximum rotation interval for translation (calculated from minimum RPM)
#define MAX_TRANSLATION_ROTATION_INTERVAL_US ((1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000)

