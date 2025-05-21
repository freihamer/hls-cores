#pragma once

#include <stdint.h>
#include "hls/streaming.hpp"

constexpr uint16_t MSG_TX_1_HEADER = 0x5AD5;
constexpr uint8_t MSG_TX_1_SIZE = 34U;

void msg_tx(
    uint8_t* msg_tx,
    hls::FIFO<uint8_t> &data_out
);