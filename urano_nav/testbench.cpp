#include <stdint.h>
#include <assert.h>

#include "hls/streaming.hpp"
#include "checksum.hpp"

#include "txframe.hpp"
#include "rxdeframe.hpp"


void test_urano_nav_tx() {

    hls::FIFO<uint8_t> data_out(TX_MSG_1_SIZE);

    uint8_t msg_1[TX_MSG_1_SIZE - 4];
    for (uint8_t i = 0; i < TX_MSG_1_SIZE - 4; ++i) {
        msg_1[i] = i;
    }

    uint8_t count = 0U;
    uint8_t bytes[TX_MSG_1_SIZE];
    bytes[count++] = TX_MSG_1_HEADER >> 8;
    bytes[count++] = TX_MSG_1_HEADER & 0xFF;
    for (uint8_t i = 0; i < TX_MSG_1_SIZE - 4; ++i) {
        bytes[count++] =  msg_1[i];
    }
    uint16_t crc = checksum(bytes, TX_MSG_1_SIZE - 2);
    bytes[count++] = (crc >> 8) & 0xFF;
    bytes[count++] = crc & 0xFF;

    urano_nav_tx(msg_1, data_out);

    uint8_t received = 0U;
    while (!data_out.empty()) {
        assert(data_out.read() == bytes[received]);
        received += 1;
    }

    assert(received == TX_MSG_1_SIZE);
}

void test_urano_nav_rx() {

    hls::FIFO<uint8_t> data_in(2);
    uint8_t msg_1[RX_MSG_1_SIZE] = {0};
    uint8_t msg_2[RX_MSG_2_SIZE] = {0};
    NavMsgStats msg_1_stats = {0};
    NavMsgStats msg_2_stats = {0};

    uint8_t bytes[RX_MSG_1_SIZE] = {0};
    uint8_t count = 0U;

    bytes[count++] = RX_MSG_1_HEADER >> 8;
    bytes[count++] = RX_MSG_1_HEADER & 0xFF;
    for (uint8_t i = 0; i < RX_MSG_1_SIZE - 4; ++i) {
        bytes[count++] = msg_1[i];
    }
    uint16_t crc = checksum(bytes, RX_MSG_1_SIZE - 2);
    bytes[count++] = (crc >> 8) & 0xFF;
    bytes[count++] = crc & 0xFF;

    for (uint8_t i = 0U; i < RX_MSG_1_SIZE; ++i) {
        data_in.write(bytes[i]);

        urano_nav_rx(data_in, msg_1, msg_2, &msg_1_stats, &msg_2_stats);
    }

    assert(msg_1_stats.valid == 1U);
    assert(msg_1_stats.invalid == 0U);
    assert(msg_1_stats.aborted == 0U);
}

int main(int argc, char** argv) {

    test_urano_nav_tx();
    test_urano_nav_rx();

    return 0;
}
