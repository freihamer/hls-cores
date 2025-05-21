#include "utils.hpp"

/* The checksum calculation implements CRC-16-CCITT (0x1021, initial 0xFFFF) */
uint16_t checksum_crc16_ccitt(const uint8_t* data, int len) {
    #pragma HLS function inline
    
        uint16_t crc = 0xFFFF;
        const uint16_t polynomial = 0x1021;
    
        for (int i = 0; i < len; ++i) {
            crc ^= (uint16_t)data[i] << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }