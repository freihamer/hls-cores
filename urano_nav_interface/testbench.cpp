#include <stdint.h>
#include "hls/streaming.hpp"
#include "checksum.hpp"

#include "txframe.hpp"
#include "rxdeframe.hpp"

int main(int argc, char** argv) {

    uint8_t errors = 0U;

    hls::FIFO<uint8_t> tx_data_out(TX_MSG_1_SIZE);
    uint8_t tx_bytes = 0U;

    uint8_t tx_rx_msg_1_payload_test[TX_MSG_1_SIZE - 4];
    for (uint8_t i = 0; i < TX_MSG_1_SIZE - 4; ++i) {
        tx_rx_msg_1_payload_test[i] = i;
    }

    uint8_t tx_msg_1_expect[TX_MSG_1_SIZE];
    tx_msg_1_expect[0] = TX_MSG_1_HEADER >> 8;
    tx_msg_1_expect[1] = TX_MSG_1_HEADER & 0xFF;
    for (uint8_t i = 0; i < TX_MSG_1_SIZE - 4; ++i) {
        tx_msg_1_expect[i + 2] = i;
    }
    uint16_t crc = checksum(tx_msg_1_expect, TX_MSG_1_SIZE - 2);
    tx_msg_1_expect[TX_MSG_1_SIZE - 2] = (crc >> 8) & 0xFF;
    tx_msg_1_expect[TX_MSG_1_SIZE - 1] = crc & 0xFF;

    uint8_t tx_msg_1_actual[TX_MSG_1_SIZE];

    urano_nav_tx(tx_rx_msg_1_payload_test, tx_data_out);
    while (!tx_data_out.empty()) {
        tx_msg_1_actual[tx_bytes++] = tx_data_out.read();
    }

    bool check = true;
    for (uint8_t i = 0; i < TX_MSG_1_SIZE; ++i) {
        if (tx_msg_1_actual[i] != tx_msg_1_expect[i]) {
            check = false;
            errors++;
            break;
        }
    }

    if (check == false) {
        printf("== MSG 1 TX Expect ===\n");
        for (uint8_t i = 0; i < TX_MSG_1_SIZE; ++i)
            printf("%02X ", tx_msg_1_expect[i]);
        printf("\n\n");
    
        printf("== MSG 1 TX Actual ===\n");
        for (uint8_t i = 0; i < TX_MSG_1_SIZE; ++i)
            printf("%02X ", tx_msg_1_actual[i]);
        printf("\n\n");
    }

    // Allocate software-side test memory
    hls::FIFO<uint8_t> rx_data_in(2);
    uint64_t rx_msg_1_timestamp = 0;
    uint64_t rx_msg_2_timestamp = 0;
    uint8_t rx_msg_1_payload[RX_MSG_1_SIZE - 4] = {0};
    uint8_t rx_msg_2_payload[RX_MSG_2_SIZE - 4] = {0};
    NavMsgStats rx_msg_1_stats = {0};
    NavMsgStats rx_msg_2_stats = {0};

    // Construct a valid receive message 1
    uint8_t rx_msg_1_test[RX_MSG_1_SIZE] = {0};
    rx_msg_1_test[0] = RX_MSG_1_HEADER >> 8;
    rx_msg_1_test[1] = RX_MSG_1_HEADER & 0xFF;
    for (uint8_t i = 2; i < RX_MSG_1_SIZE - 2; ++i) {
        rx_msg_1_test[i] = i;
    }
    crc = checksum(rx_msg_1_test, RX_MSG_1_SIZE - 2);
    rx_msg_1_test[RX_MSG_1_SIZE - 2] = (crc >> 8) & 0xFF;
    rx_msg_1_test[RX_MSG_1_SIZE - 1] = crc & 0xFF;

    // Construct a valid receive message 2
    uint8_t rx_msg_2_test[RX_MSG_2_SIZE] = {0};
    rx_msg_2_test[0] = RX_MSG_2_HEADER >> 8;
    rx_msg_2_test[1] = RX_MSG_2_HEADER & 0xFF;
    for (uint8_t i = 2; i < RX_MSG_2_SIZE - 2; ++i) {
        rx_msg_2_test[i] = i;
    }
    crc = checksum(rx_msg_2_test, RX_MSG_2_SIZE - 2);
    rx_msg_2_test[RX_MSG_2_SIZE - 2] = (crc >> 8) & 0xFF;
    rx_msg_2_test[RX_MSG_2_SIZE - 1] = crc & 0xFF;

    // Feed message byte-by-byte into the FIFO
    for (uint8_t i = 0; i < RX_MSG_1_SIZE + RX_MSG_2_SIZE; ++i) {
        if(i < RX_MSG_1_SIZE)
            rx_data_in.write(rx_msg_1_test[i]);
        else
            rx_data_in.write(rx_msg_2_test[i - RX_MSG_1_SIZE]);

        // Call the module once per byte (as if one clock cycle)
        urano_nav_rx(rx_data_in,
                   &rx_msg_1_timestamp,
                   &rx_msg_2_timestamp,
                   rx_msg_1_payload,
                   rx_msg_2_payload,
                   &rx_msg_1_stats,
                   &rx_msg_2_stats);
    }

    if (((rx_msg_1_stats.valid == 1U) && (rx_msg_1_stats.invalid == 0U) && (rx_msg_1_stats.aborted == 0U)) == false) {
        errors++;

        printf("== RX MSG 1 Stats ==\n");
        printf("Valid:   %u\n", rx_msg_1_stats.valid);
        printf("Invalid: %u\n", rx_msg_1_stats.invalid);
        printf("Aborted: %u\n", rx_msg_1_stats.aborted);
        printf("Timestamp: %lu\n", (unsigned long)rx_msg_1_timestamp);

        printf("== RX MSG 1 Payload ===\n");
        for (uint8_t i = 0; i < RX_MSG_1_SIZE - 4; ++i)
            printf("%02X ", rx_msg_1_payload[i]);
        printf("\n\n");
    }
    
    
    if (((rx_msg_2_stats.valid == 1U) && (rx_msg_2_stats.invalid == 0U) && (rx_msg_2_stats.aborted == 0U)) == false) {
        errors++;

        printf("== RX MSG 2 Stats ==\n");
        printf("Valid:   %u\n", rx_msg_2_stats.valid);
        printf("Invalid: %u\n", rx_msg_2_stats.invalid);
        printf("Aborted: %u\n", rx_msg_2_stats.aborted);
        printf("Timestamp: %lu\n", (unsigned long)rx_msg_2_timestamp);

        printf("== RX MSG 2 Payload ===\n");
        for (uint8_t i = 0; i < RX_MSG_2_SIZE - 4; ++i)
            printf("%02X ", rx_msg_2_payload[i]);
        printf("\n\n");
    }

    return errors;
}
