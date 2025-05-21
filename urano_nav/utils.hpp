#pragma once

#include <stdint.h>

uint16_t checksum_crc16_ccitt(const uint8_t* data, int len);
