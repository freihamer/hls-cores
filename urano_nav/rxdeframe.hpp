#pragma once

#include <stdint.h>
#include "hls/streaming.hpp"

struct NavMsgStats {
    uint32_t valid;
    uint32_t invalid;
    uint32_t aborted;
};

constexpr uint16_t RX_MSG_1_HEADER = 0xA0A5;
constexpr uint8_t RX_MSG_1_SIZE = 30U;

constexpr uint16_t RX_MSG_2_HEADER = 0xAA55;
constexpr uint8_t RX_MSG_2_SIZE = 160U;

void urano_nav_rx(
    hls::FIFO<uint8_t> &data_in,             
    uint8_t* msg_1_payload,
    uint8_t* msg_2_payload,
    NavMsgStats* msg_1_stats,
    NavMsgStats* msg_2_stats
);