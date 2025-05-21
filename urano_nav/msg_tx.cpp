#include "msg_tx.hpp"
#include "utils.hpp"

void msg_tx(
    uint8_t* msg_tx,
    hls::FIFO<uint8_t> &data_out
) {
#pragma HLS function top
#pragma HLS interface control type(axi_target)
#pragma HLS interface argument(msg_tx) type(axi_target) num_elements(MSG_TX_1_SIZE - 4)

    uint8_t buffer[MSG_TX_1_SIZE];

    buffer[0] = MSG_TX_1_HEADER >> 8;
    buffer[1] = MSG_TX_1_HEADER & 0xFF;

#pragma HLS loop unroll
    for (uint8_t i = 2; i < MSG_TX_1_SIZE - 2; ++i)
        buffer[i] = msg_tx[i - 2];

    uint16_t crc = checksum_crc16_ccitt(buffer, MSG_TX_1_SIZE - 2);
    buffer[MSG_TX_1_SIZE - 2] = (crc >> 8) & 0xFF;
    buffer[MSG_TX_1_SIZE - 1] = crc & 0xFF;
    
    for (uint8_t i = 0; i < MSG_TX_1_SIZE; ++i)
        data_out.write(buffer[i]);
}