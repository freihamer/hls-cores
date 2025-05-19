#include <stdint.h>
#include <hls_stream.h>

// The timeout in clock cycles, between any two expected receive bytes.
#define DATA_IN_TIMEOUT 5000U // 100us @ 50MHz

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


#define NAV_MSG_TX_1_HEADER_1 0xD5
#define NAV_MSG_TX_1_HEADER_2 0x5A
#define NAV_MSG_TX_1_SIZE 34U

void nav_msg_tx(
    uint8_t* msg_tx,
    hls::FIFO<uint8_t> &data_out
) {
#pragma HLS function top
#pragma HLS interface control type(axi_target)

    uint8_t buffer[NAV_MSG_TX_1_SIZE];

    buffer[0] = NAV_MSG_TX_1_HEADER_1;
    buffer[1] = NAV_MSG_TX_1_HEADER_2;
    for (int i = 2; i < NAV_MSG_TX_1_SIZE; i++)
        buffer[i] = msg_tx[i];
    uint16_t crc = checksum_crc16_ccitt(buffer, NAV_MSG_TX_1_SIZE - 2);
    buffer[NAV_MSG_TX_1_SIZE - 2] = (crc >> 8) & 0xFF;
    buffer[NAV_MSG_TX_1_SIZE - 1] = crc & 0xFF;
    
    for (int i = 0; i < NAV_MSG_TX_1_SIZE; i++)
        data_out.write(buffer[i]);
}


#define NAV_MSG_RX_1_HEADER_1 0xA0
#define NAV_MSG_RX_1_HEADER_2 0xA5
#define NAV_MSG_RX_1_SIZE 30U

#define NAV_MSG_RX_2_HEADER_1 0xAA
#define NAV_MSG_RX_2_HEADER_2 0x55
#define NAV_MSG_RX_2_SIZE 160U

void nav_msg_rx(
    hls::FIFO<uint8_t> &data_in                   
    uint8_t* msg_rx_1,
    uint8_t* msg_rx_2,
    bool* msg_rx_1_valid,
    bool* msg_rx_2_valid,
    
) {
#pragma HLS function top
#pragma HLS interface control type(simple)
#pragma HLS interface argument(rx_byte) type(simple) stable(false)
#pragma HLS interface argument(rx_ready) type(simple) stable(false)
#pragma HLS interface argument(rxd_1) type(memory) num_elements(RXD_1_SIZE)
#pragma HLS interface argument(rxd_2) type(memory) num_elements(RXD_2_SIZE)

    static enum State {
        IDLE,
        HEADER_1,
        HEADER_2,
        RECEIVING,
        PAUSED,
        COMPLETED
    } msg_rx_1_state = IDLE, msg_rx_2_state = IDLE;

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