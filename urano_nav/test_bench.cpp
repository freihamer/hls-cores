#include <stdint.h>
#include "hls/streaming.hpp"
#include "utils.hpp"

#include "msg_tx.hpp"
#include "msg_rx.hpp"

int main(int argc, char** argv) {

    hls::FIFO<uint8_t> data_out(MSG_TX_1_SIZE);
    uint8_t bytes_sent = 0U;

    uint8_t msg_tx_test[MSG_TX_1_SIZE - 4];
    for (uint8_t i = 0; i < MSG_TX_1_SIZE - 4; ++i) {
        msg_tx_test[i] = i;
    }

    uint8_t msg_tx_expect[MSG_TX_1_SIZE];
    msg_tx_expect[0] = MSG_TX_1_HEADER >> 8;
    msg_tx_expect[1] = MSG_TX_1_HEADER & 0xFF;
    for (uint8_t i = 0; i < MSG_TX_1_SIZE - 4; ++i) {
        msg_tx_expect[i + 2] = i;
    }
    uint16_t crc = checksum_crc16_ccitt(msg_tx_expect, MSG_TX_1_SIZE - 2);
    msg_tx_expect[MSG_TX_1_SIZE - 2] = (crc >> 8) & 0xFF;
    msg_tx_expect[MSG_TX_1_SIZE - 1] = crc & 0xFF;

    uint8_t msg_tx_actual[MSG_TX_1_SIZE];

    msg_tx(msg_tx_test, data_out);
    while (!data_out.empty()) {
        msg_tx_actual[bytes_sent++] = data_out.read();
    }

    printf("== MSG 1 TX Expect ===\n");
    for (uint8_t i = 0; i < MSG_TX_1_SIZE; ++i)
        printf("%02X ", msg_tx_expect[i]);
    printf("\n\n");

    printf("== MSG 1 TX Actual ===\n");
    for (uint8_t i = 0; i < MSG_TX_1_SIZE; ++i)
        printf("%02X ", msg_tx_actual[i]);
    printf("\n\n");

    uint8_t transmit_errors = 0U;
    for (uint8_t i = 0; i < MSG_TX_1_SIZE; ++i) {
        if (msg_tx_actual[i] != msg_tx_expect[i])
            transmit_errors++;
    }

    // Allocate software-side test memory
    hls::FIFO<uint8_t> data_in(2);
    uint64_t msg_1_timestamp = 0;
    uint64_t msg_2_timestamp = 0;
    uint8_t msg_1_body[MSG_1_SIZE - 4] = {0};
    uint8_t msg_2_body[MSG_2_SIZE - 4] = {0};
    NavMsgStats msg_1_stats = {0};
    NavMsgStats msg_2_stats = {0};

    // Construct a valid MSG 1 message
    uint8_t msg_1_test[MSG_1_SIZE] = {0};
    msg_1_test[0] = MSG_1_HEADER >> 8;
    msg_1_test[1] = MSG_1_HEADER & 0xFF;
    for (uint8_t i = 2; i < MSG_1_SIZE - 2; ++i) {
        msg_1_test[i] = i;
    }
    crc = checksum_crc16_ccitt(msg_1_test, MSG_1_SIZE - 2);
    msg_1_test[MSG_1_SIZE - 2] = (crc >> 8) & 0xFF;
    msg_1_test[MSG_1_SIZE - 1] = crc & 0xFF;

    // Construct a valid MSG 2 message
    uint8_t msg_2_test[MSG_2_SIZE] = {0};
    msg_2_test[0] = MSG_2_HEADER >> 8;
    msg_2_test[1] = MSG_2_HEADER & 0xFF;
    for (uint8_t i = 2; i < MSG_2_SIZE - 2; ++i) {
        msg_2_test[i] = i;
    }
    crc = checksum_crc16_ccitt(msg_2_test, MSG_2_SIZE - 2);
    msg_2_test[MSG_2_SIZE - 2] = (crc >> 8) & 0xFF;
    msg_2_test[MSG_2_SIZE - 1] = crc & 0xFF;

    // Feed message byte-by-byte into the FIFO
    for (uint8_t i = 0; i < MSG_1_SIZE + MSG_2_SIZE; ++i) {
        if(i < MSG_1_SIZE)
            data_in.write(msg_1_test[i]);
        else
            data_in.write(msg_2_test[i-MSG_1_SIZE]);

        // Call the module once per byte (as if one clock cycle)
        msg_rx(data_in,
                   &msg_1_timestamp,
                   &msg_2_timestamp,
                   msg_1_body,
                   msg_2_body,
                   &msg_1_stats,
                   &msg_2_stats);
    }

    // Check results
    printf("== MSG 1 STATS ==\n");
    printf("Valid:   %u\n", msg_1_stats.valid);
    printf("Invalid: %u\n", msg_1_stats.invalid);
    printf("Aborted: %u\n", msg_1_stats.aborted);
    printf("Timestamp: %lu\n", (unsigned long)msg_1_timestamp);

    printf("== MSG 1 Body ===\n");
    for (uint8_t i = 0; i < MSG_1_SIZE - 4; ++i)
        printf("%02X ", msg_1_body[i]);
    printf("\n\n");
    
    printf("== MSG 2 STATS ==\n");
    printf("Valid:   %u\n", msg_2_stats.valid);
    printf("Invalid: %u\n", msg_2_stats.invalid);
    printf("Aborted: %u\n", msg_2_stats.aborted);
    printf("Timestamp: %lu\n", (unsigned long)msg_2_timestamp);

    printf("== MSG 2 Body ===\n");
    for (uint8_t i = 0; i < MSG_2_SIZE - 4; ++i)
        printf("%02X ", msg_2_body[i]);
    
    uint8_t errors = !((transmit_errors == 0) &&
        (msg_1_stats.valid == 1U) &&
        (msg_1_stats.invalid == 0U) &&
        (msg_1_stats.aborted == 0U) &&
        (msg_2_stats.valid == 1U) &&
        (msg_2_stats.invalid == 0U) &&
        (msg_2_stats.aborted == 0U));

    return errors;
}
