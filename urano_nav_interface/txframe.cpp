#include "txframe.hpp"
#include "checksum.hpp"

void urano_nav_tx(
    uint8_t* msg_1_payload,
    hls::FIFO<uint8_t> &data_out
) {
#pragma HLS function top
#pragma HLS interface control type(axi_target)
#pragma HLS interface argument(msg_1_payload) type(axi_target) num_elements(TX_MSG_1_SIZE - 4) dma(false) requires_copy_in(true)

    uint8_t buffer[TX_MSG_1_SIZE];

    buffer[0] = TX_MSG_1_HEADER >> 8;
    buffer[1] = TX_MSG_1_HEADER & 0xFF;

#pragma HLS loop unroll
    for (uint8_t i = 2; i < TX_MSG_1_SIZE - 2; ++i)
        buffer[i] = msg_1_payload[i - 2];

    uint16_t crc = checksum(buffer, TX_MSG_1_SIZE - 2);
    buffer[TX_MSG_1_SIZE - 2] = (crc >> 8) & 0xFF;
    buffer[TX_MSG_1_SIZE - 1] = crc & 0xFF;
    
    for (uint8_t i = 0; i < TX_MSG_1_SIZE; ++i)
        data_out.write(buffer[i]);
}