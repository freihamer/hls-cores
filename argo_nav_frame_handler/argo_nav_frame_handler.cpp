#include <stdint.h>
#include <hls_stream.h>

// The timeout in clock cycles, between any two expected receive bytes.
#define RXD_TIMEOUT 5000U // 100us @ 50MHz

#define RXD_1_HEADER_1 0xA0
#define RXD_1_HEADER_2 0xA5
#define RXD_1_SIZE 30U

#define RXD_2_HEADER_1 0xAA
#define RXD_2_HEADER_2 0x55
#define RXD_2_SIZE 160U

#define TXD_1_HEADER_1 0xD5
#define TXD_1_HEADER_2 0x5A
#define TXD_1_SIZE 34U

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

void argo_nav_frame_handler(
    uint8_t rx_byte,                   // in
    bool rx_ready,                     // in   
    uint8_t* txd_1,                    // in
    bool txd_1_ready,                  // in                    
    uint8_t* rxd_1,                    // out
    uint8_t* rxd_2,                    // out
    bool* rxd_1_valid,                 // out
    bool* rxd_2_valid,                 // out
    hls::FIFO<uint8_t> &tx_fifo        // out
) {
#pragma HLS function top
#pragma HLS interface control type(simple)
#pragma HLS interface argument(rx_byte) type(simple) stable(false)
#pragma HLS interface argument(rx_ready) type(simple) stable(false)
#pragma HLS interface argument(rxd_1) type(memory) num_elements(RXD_1_SIZE)
#pragma HLS interface argument(rxd_2) type(memory) num_elements(RXD_2_SIZE)

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

    if (txd_1_ready == true) {
        txd_1_buf[TXD_1_SIZE];
        txd_1_buf[0] = TXD_1_HEADER_1;
        txd_1_buf[1] = TXD_1_HEADER_2;
        for (int i = 2; i < TXD_1_SIZE; i++)
            txd_1_buf[i] = txd_1[i];
        uint16_t crc = checksum_crc16_ccitt(txd_1_buf, TXD_1_SIZE - 2);
        txd_1_buf[TXD_1_SIZE - 2] = (crc >> 8) & 0xFF;
        txd_1_buf[TXD_1_SIZE - 1] = crc & 0xFF;

        for (int i = 0; i < TXD_1_SIZE; i++)
            tx_fifo.write(txd_1_buf[i]);
    }
}