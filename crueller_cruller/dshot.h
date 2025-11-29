#ifndef DSHOT_H
#define DSHOT_H

#include <driver/rmt.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// DSHOT PROTOCOL CONFIGURATION
// ============================================================================
// DShot is a digital ESC communication protocol. These constants define
// the protocol timing and packet structure.

// Clock divider for RMT peripheral (lower = faster, but may cause timing issues)
constexpr auto DSHOT_CLK_DIVIDER = 8;

// Packet structure
constexpr auto DSHOT_PACKET_LENGTH = 17;      // Total bits in a DShot packet (16 data + 1 pause)
constexpr auto DSHOT_THROTTLE_MIN = 48;       // Minimum throttle value (0-47 are special commands)
constexpr auto DSHOT_THROTTLE_MAX = 2047;     // Maximum throttle value (11-bit)
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;  // Null packet for initialization
constexpr auto DSHOT_PAUSE = 21;               // Pause duration between packets (in RMT ticks)
constexpr auto DSHOT_PAUSE_BIT = 16;           // Position of pause bit in packet array

// DShot speed modes (higher numbers = faster communication, requires faster ESC)
typedef enum dshot_mode_e {
    DSHOT_OFF,      // Protocol disabled
    DSHOT150,       // 150kHz (slowest, most compatible)
    DSHOT300,       // 300kHz (medium speed)
    DSHOT600,       // 600kHz (fast, recommended for most ESCs)
    DSHOT1200       // 1200kHz (fastest, requires high-speed ESC)
} dshot_mode_t;

// DShot packet structure (16-bit packet with checksum)
typedef struct dshot_packet_s {
    uint16_t throttle_value : 11;      // Throttle value (0-2047)
    uint8_t telemetric_request : 1;     // Request telemetry data (0 = no, 1 = yes)
    uint16_t checksum : 4;              // CRC checksum for error detection
} dshot_packet_t;

// DShot configuration structure
typedef struct dshot_config_s {
    dshot_mode_t mode;                 // DShot speed mode
    bool is_bidirectional;             // Bidirectional DShot (allows ESC feedback)
    gpio_num_t gpio_num;               // GPIO pin number
    uint8_t pin_num;                    // Pin number (duplicate for compatibility)
    rmt_channel_t rmt_channel;         // RMT channel for signal generation
    uint8_t mem_block_num;              // RMT memory block number
    uint16_t ticks_per_bit;             // RMT ticks per data bit (calculated from mode)
    uint8_t clk_div;                    // Clock divider
    uint16_t ticks_zero_high;           // High pulse duration for '0' bit
    uint16_t ticks_zero_low;            // Low pulse duration for '0' bit
    uint16_t ticks_one_high;            // High pulse duration for '1' bit
    uint16_t ticks_one_low;             // Low pulse duration for '1' bit
} dshot_config_t;

// DShotRMT class for ESC control via RMT peripheral
class DShotRMT {
public:
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel);
    ~DShotRMT();
    
    // Initialize DShot protocol with specified mode
    bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);
    
    // Send throttle value to ESC (0-2047, or use throttlePercentToDShot for 0-100%)
    void sendThrottleValue(uint16_t throttle_value);

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH];
    rmt_config_t dshot_tx_rmt_config;
    dshot_config_t dshot_config;

    rmt_item32_t *buildTxRmtItem(uint16_t parsed_packet);
    uint16_t calculateCRC(const dshot_packet_t &dshot_packet);
    uint16_t parseRmtPaket(const dshot_packet_t &dshot_packet);
    void sendRmtPaket(const dshot_packet_t &dshot_packet);
};

// Helper functions for throttle conversion
uint16_t throttleToDShot(int throttle);
uint16_t throttlePercentToDShot(uint16_t throttlePercent);

#endif // DSHOT_H

