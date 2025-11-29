#include "dshot.h"
#include <Arduino.h>

// ============================================================================
// DSHOTRMT CLASS IMPLEMENTATION
// ============================================================================

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel) {
    dshot_config.gpio_num = gpio;
    dshot_config.pin_num = static_cast<uint8_t>(gpio);
    dshot_config.rmt_channel = rmtChannel;
    dshot_config.mem_block_num = static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel));
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::~DShotRMT() {
    rmt_driver_uninstall(dshot_config.rmt_channel);
}

bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional) {
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

void DShotRMT::sendThrottleValue(uint16_t throttle_value) {
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

rmt_item32_t *DShotRMT::buildTxRmtItem(uint16_t parsed_packet) {
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

uint16_t DShotRMT::calculateCRC(const dshot_packet_t &dshot_packet) {
    const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
    if (dshot_config.is_bidirectional) {
        const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
        return (~intermediate_result) & 0x0F;
    } else {
        return (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    }
}

uint16_t DShotRMT::parseRmtPaket(const dshot_packet_t &dshot_packet) {
    uint16_t crc = calculateCRC(dshot_packet);
    uint16_t parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
    parsedRmtPaket = (parsedRmtPaket << 4) | crc;
    return parsedRmtPaket;
}

void DShotRMT::sendRmtPaket(const dshot_packet_t &dshot_packet) {
    buildTxRmtItem(parseRmtPaket(dshot_packet));
    rmt_write_items(dshot_tx_rmt_config.channel, dshot_tx_rmt_item, DSHOT_PACKET_LENGTH, false);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

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

