#pragma once

#include <stdint.h>
#include "hls/streaming.hpp"

struct NavMsgStats {
    uint32_t valid;
    uint32_t invalid;
    uint32_t aborted;
};

constexpr uint16_t  MSG_1_HEADER = 0xA0A5;
constexpr uint8_t MSG_1_SIZE = 30U;
constexpr uint16_t MSG_2_HEADER = 0xAA55;
constexpr uint8_t MSG_2_SIZE = 160U;

void msg_rx(
    hls::FIFO<uint8_t> &data_in,
    uint64_t* msg_1_timestamp,
    uint64_t* msg_2_timestamp,                  
    uint8_t* msg_1_body,
    uint8_t* msg_2_body,
    NavMsgStats* msg_1_stats,
    NavMsgStats* msg_2_stats
);