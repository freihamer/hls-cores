#pragma once

#include <stdint.h>
#include "hls/streaming.hpp"

constexpr uint16_t TX_MSG_1_HEADER = 0x5AD5;
constexpr uint8_t TX_MSG_1_SIZE = 34U;

void urano_nav_tx(
    uint8_t* msg_1_payload,
    hls::FIFO<uint8_t> &data_out
);