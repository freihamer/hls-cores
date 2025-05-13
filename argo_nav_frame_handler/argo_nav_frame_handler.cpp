#include <stdint.h>
#include <hls_stream.h>

#define F1_HEADER_1 0xA0
#define F1_HEADER_2 0xA5
#define F1_SIZE 30

#define F2_HEADER_1 0xAA
#define F2_HEADER_2 0x55
#define F2_SIZE 160

#define RECEIVE_TIMEOUT 50000U // Receive timeout in clock cycles, between any two expected bytes in a frame. This translates to 1ms @ 50MHz

typedef struct {
    uint8_t data[1024]; // 1KB
    uint32_t valid_count;
    uint32_t error_count;
} Frame;

/* The checksum calculation implements CRC-16-CCITT (0x1021, initial 0xFFFF) */
uint16_t checksum_crc16_ccitt(const uint8_t* data, int len) {
    
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

enum FSMState {
    IDLE,
    F1_HEADER_1,
    F1_HEADER_2,
    F1_RECEIVING,
    F1_COMPLETED,
    F2_HEADER_1,
    F2_HEADER_2,
    F2_RECEIVING
};

void uart_frame_handler(
    uint8_t rx_byte,
    bool rx_valid,
    Frame* out_f1,
    Frame* out_f2
) {

#pragma HLS interface control type(axi_target)

    static FSMState state = IDLE;
    static uint8_t f1_buf[F1_SIZE];
    static uint8_t f2_buf[F2_SIZE];
    static uint8_t f1_bytes = 0U;
    static uint8_t f2_bytes = 0U;
    static uint16_t rx_gap = 0U; // The number of clock cycles spent waiting for the next byte of a sequence


    if (rx_valid == false) {
        if (state != IDLE && ++rx_gap >= RECEIVE_TIMEOUT) {
            rx_gap = 0;
            f1_bytes = 0U;
            f2_bytes = 0U;
            state = IDLE;
        }
        return;
    }

    switch (state) {
        case IDLE:
            if (rx_byte == F1_HEADER_1) {
                state = F1_HEADER_1;
            }
            break;

        case F1_HEADER_1:
            if (rx_byte == F1_HEADER_2) {
                f1_buf[0] = F1_HEADER_1;
                f1_buf[1] = F1_HEADER_2;
                f1_bytes = 2;
                state = F1_RECEIVING;
            } else {
                state = IDLE;
            }
            break;

        case F1_RECEIVING:
            f1_buf[f1_bytes++] = rx_byte;
            if (f1_bytes == F1_SIZE) {
                uint16_t expected = (f1_buf[F1_SIZE - 2] << 8) | f1_buf[F1_SIZE - 1];
                uint16_t computed = checksum_crc16_ccitt(f1_buf, F1_SIZE - 2);
                if (computed == expected) {
                    for (int i = 0; i < F1_SIZE; ++i)
                        out_f1->data[i] = f1_buf[i];
                    out_f1->valid_count += 1;
                } else {
                    out_f1->error_count += 1;
                }
                state = F2_WAIT_HEADER_1;
            }
            break;

        case F2_WAIT_HEADER_1:
            if (f1_completed && rx_byte == 0xAA)
                state = F2_WAIT_HEADER_2;
            else
                state = IDLE;  // If F2 doesn't start right after F1, reset
            break;

        case F2_WAIT_HEADER_2:
            if (rx_byte == 0x55) {
                f2_buf[0] = 0xAA;
                f2_buf[1] = 0x55;
                f2_bytes = 2;
                state = F2_RECEIVING;
            } else {
                state = IDLE;
            }
            break;

        case F2_RECEIVING:
            f2_buf[f2_bytes++] = rx_byte;
            if (f2_bytes == F2_SIZE) {
                uint16_t expected = (f2_buf[F2_SIZE - 2] << 8) | f2_buf[F2_SIZE - 1];
                uint16_t computed = checksum_crc16_ccitt(f2_buf, F2_SIZE - 2);
                if (computed == expected) {
                    for (int i = 0; i < F2_SIZE; ++i)
                        out_f2->data[i] = f2_buf[i];
                    out_f2->count += 1;
                }
                f1_completed = false;
                state = IDLE;
            }
            break;
    }
}