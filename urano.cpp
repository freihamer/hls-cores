#include <stdint.h>
#include <hls_stream.h>

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


#define MSG_TX_1_HEADER_1 0xD5
#define MSG_TX_1_HEADER_2 0x5A
#define MSG_TX_1_SIZE 34U

void nav_msg_tx(
    uint8_t* msg_tx,
    hls::FIFO<uint8_t> &data_out
) {
#pragma HLS function top
#pragma HLS interface control type(simple)

    uint8_t buffer[MSG_TX_1_SIZE];

    buffer[0] = MSG_TX_1_HEADER_1;
    buffer[1] = MSG_TX_1_HEADER_2;

#pragma HLS loop unroll
    for (int i = 2; i < MSG_TX_1_SIZE - 2; ++i)
        buffer[i] = msg_tx[i - 2];

    uint16_t crc = checksum_crc16_ccitt(buffer, MSG_TX_1_SIZE - 2);
    buffer[MSG_TX_1_SIZE - 2] = (crc >> 8) & 0xFF;
    buffer[MSG_TX_1_SIZE - 1] = crc & 0xFF;
    
    for (int i = 0; i < MSG_TX_1_SIZE; ++i)
        data_out.write(buffer[i]);
}


enum class ReceiverState {
    Idle,
    Starting,
    Receiving,
    Paused,
    Completed
};

class NavMsgReceiver {
public:
    NavMsgReceiver(uint16_t header, uint8_t size) : 
        header(header),
        size(size),
        counter(0),
        buffer{0},
        bytes_count(0),
        state(ReceiverState::Idle) {}

    void reset() {
        counter = 0;
        bytes_count = 0;
        state = ReceiverState::Idle;
    }

    ReceiverState get_state() const {
        return state;
    }

    ReceiverState process(hls::FIFO<uint8_t> &data_in) {
        if (data_in.empty()) return;

        uint8_t byte_rx = data_in.read();

        switch (state) {
            case ReceiverState::Idle:
                if (byte_rx == (header >> 8)) {
                    buffer[counter++] = byte_rx;
                    state = ReceiverState::Starting;
                }
                break;

            case ReceiverState::Starting:
                if (byte_rx == (header & 0xFF)) {
                    buffer[counter++] = byte_rx;
                    state = ReceiverState::Receiving;
                } else {
                    reset();
                }
                break;

            case ReceiverState::Receiving:
                buffer[counter++] = byte_rx;
                if (counter == size) {
                    uint16_t checksum_expected = (buffer[size - 2] << 8) | buffer[size - 1];
                    uint16_t checksum_computed = checksum_crc16_ccitt(buffer, size - 2);
                    if (checksum_expected == checksum_computed) {
                        state = ReceiverState::Completed;
                    } else {
                        reset();
                    }
                }
                break;

            case ReceiverState::Paused:
            case ReceiverState::Completed:
                break;
        }

        return state;
    }

private:

    ReceiverState state;
    uint8_t counter = 0;
    uint8_t buffer[256] = {0};
    uint16_t header;
    uint8_t size;
    
};

void nav_msg_rx(
    hls::FIFO<uint8_t> &data_in                   
    uint8_t* msg_rx_1,
    uint8_t* msg_rx_2
) {
#pragma HLS function top
#pragma HLS interface control type(simple)
#pragma HLS interface argument(msg_rx_1) type(memory) num_elements(MSG_RX_1_SIZE)
#pragma HLS interface argument(msg_rx_2) type(memory) num_elements(MSG_RX_2_SIZE)

    static NavMsgReceiver msg_receiver_1(0xA5A0, 30);
    static NavMsgReceiver msg_receiver_2(0x55AA, 160);

    if (msg_receiver_1.get_state() == ReceiverState::Completed) {
        msg_receiver_2.process(data_in);
    } else {
        msg_receiver_1.process(data_in);
    }


    static uint16_t gap_cycles = 0U; // The number of clock cycles spent waiting for the next byte_rx of a sequence

    if (data_in.empty()) {
        if (msg_rx_1_state != IDLE && ++gap_cycles >= 5000U) {
            gap_cycles = 0;
            msg_rx_1_cnt = 0U;
            msg_rx_1_state = IDLE;
        }
        return;
    }

    uint8_t byte_rx = data_in.read();

    switch (msg_rx_1_state) {
        case IDLE:
            if (byte_rx == MSG_RX_1_HEADER_1) {
                msg_rx_1_buf[msg_rx_1_cnt++] = byte_rx;
                msg_rx_1_state = STARTING;
            }
            break;

        case STARTING:
            if (byte_rx == MSG_RX_1_HEADER_2) {
                msg_rx_1_buf[msg_rx_1_cnt++] = byte_rx;
                msg_rx_1_state = RECEIVING;
            } else {
                msg_rx_1_cnt = 0U;
                msg_rx_1_state = IDLE;
            }
            break;

        case RECEIVING:
            msg_rx_1_buf[msg_rx_1_cnt++] = byte_rx;
            if (msg_rx_1_cnt == MSG_RX_1_SIZE) {
                uint16_t checksum_expected = (msg_rx_1_buf[MSG_RX_1_SIZE - 2] << 8) | msg_rx_1_buf[MSG_RX_1_SIZE - 1];
                uint16_t checksum_computed = checksum_crc16_ccitt(msg_rx_1_buf, MSG_RX_1_SIZE - 2);
                if (checksum_expected == checksum_computed) {
                    for (int i = 2; i < MSG_RX_1_SIZE - 2; ++i)
                        msg_rx_1[i - 2] = msg_rx_1_buf[i];
                }
                msg_rx_1_state = COMPLETED;
            }
            break;

    }
}