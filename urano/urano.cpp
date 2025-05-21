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


enum class ReceiverState {
    Idle,
    Starting,
    Receiving,
    Completed
};

class NavMsgReceiver {
public:
    NavMsgReceiver(uint16_t header, uint8_t size) : 
        header_(header),
        size_(size),
        counter_(0),
        buffer_{0},
        state_(ReceiverState::Idle) {}

    void reset() {
        counter_ = 0;
        state_ = ReceiverState::Idle;
    }

    ReceiverState get_state() const {
        return state_;
    }

    const uint8_t (&get_buffer() const)[256] {
        return buffer_;
    }

    uint8_t get_counter() const {
        return counter_;
    }

    ReceiverState process(uint8_t byte_rx) {
        switch (state_) {
            case ReceiverState::Idle:
                if (byte_rx == (header_ >> 8)) {
                    buffer_[counter_++] = byte_rx;
                    state_ = ReceiverState::Starting;
                }
                break;

            case ReceiverState::Starting:
                if (byte_rx == (header_ & 0xFF)) {
                    buffer_[counter_++] = byte_rx;
                    state_ = ReceiverState::Receiving;
                } else {
                    reset();
                }
                break;

            case ReceiverState::Receiving:
                buffer_[counter_++] = byte_rx;
                if (counter_ == size_) {
                    state_ = ReceiverState::Completed;
                }
                break;

            case ReceiverState::Completed:
                break;
        }

        return state_;
    }

private:

    ReceiverState state_;
    uint8_t counter_ = 0;
    uint8_t buffer_[256];
    uint16_t header_;
    uint8_t size_;
    
};

struct NavMsgStats {
    uint32_t valid;
    uint32_t invalid;
    uint32_t aborted;
};

#define MSG_1_HEADER 0xA5A0
#define MSG_1_SIZE 30U
#define MSG_2_HEADER 0x55AA
#define MSG_2_SIZE 160U

void nav_msg_rx(
    hls::FIFO<uint8_t> &data_in,
    uint64_t* msg_1_timestamp,
    uint64_t* msg_2_timestamp,                  
    uint8_t* msg_1_body,
    uint8_t* msg_2_body,
    NavMsgStats* msg_1_stats,
    NavMsgStats* msg_2_stats
) {
#pragma HLS function top
#pragma HLS interface control type(simple)
#pragma HLS interface argument(msg_1_timestamp) type(memory)
#pragma HLS interface argument(msg_2_timestamp) type(memory)
#pragma HLS interface argument(msg_1_body) type(memory) num_elements(MSG_1_SIZE - 4)
#pragma HLS interface argument(msg_2_body) type(memory) num_elements(MSG_2_SIZE - 4)

    static uint64_t clock_cycles = 0U;
    static uint16_t gap_tracker = 0U; // The number of clock clock_cycles spent waiting for the next byte_rx of a sequence

    static NavMsgReceiver msg_1_receiver(MSG_1_HEADER, MSG_1_SIZE);
    static NavMsgReceiver msg_2_receiver(MSG_2_HEADER, MSG_2_SIZE);

    
    clock_cycles++;

    if (data_in.empty()) {
        if (msg_1_receiver.get_State() != ReceiverState::Idle && ++gap_tracker >= 30000U) {
            /* The receiver is expecting the next byte, but the gap tracker has reached the threshold, indicating a timeout.
             * The gap tracker threshold is set to 30000 clock cycles, which corresponds to 100us at a 300MHz clock frequency.
            */
            msg_1_receiver.reset();
            msg_1_stats->aborted++;
            gap_tracker = 0U;
        } else {
            gap_tracker++;
        }
        return;
    }

    uint8_t byte_rx = data_in.read();

    if (msg_1_receiver.get_state() == ReceiverState::Completed) {
        if (msg_2_receiver.process(byte_rx) == ReceiverState::Completed) {
            const auto& buffer = msg_2_receiver.get_buffer();
            uint16_t expected = (buffer[MSG_2_SIZE - 2] << 8) | buffer[MSG_2_SIZE - 1];
            uint16_t computed = checksum_crc16_ccitt(buffer, MSG_2_SIZE - 2);
            if (expected == computed) {
#pragma HLS loop unroll
                for (int i = 2; i < MSG_2_SIZE - 2; ++i) {
                    msg_2_body[i - 2] = buffer[i];
                }

                *msg_2_timestamp = clock_cycles;
                msg_2_stats->valid++;
            } else {
                msg_2_stats->invalid++;
            }

            msg_1_receiver.reset();
        }

    } else if (msg_1_receiver.process(byte_rx) == ReceiverState::Completed) {
        const auto& buffer = msg_1_receiver.get_buffer();
        uint16_t expected = (buffer[MSG_1_SIZE - 2] << 8) | buffer[MSG_1_SIZE - 1];
        uint16_t computed = checksum_crc16_ccitt(buffer, MSG_1_SIZE - 2);
        if (expected == computed) {
#pragma HLS loop unroll
            for (int i = 2; i < MSG_1_SIZE - 2; ++i) {
                msg_1_body[i - 2] = buffer[i];
            }

            *msg_1_timestamp = clock_cycles;
            msg_1_stats->valid++;
        } else {
            msg_1_stats->invalid++;
        } 
    }
}


#define MSG_TX_1_HEADER 0x5AD5
#define MSG_TX_1_SIZE 34U

void nav_msg_tx(
    uint8_t* msg_tx,
    hls::FIFO<uint8_t> &data_out
) {
#pragma HLS function top
#pragma HLS interface control type(simple)

    uint8_t buffer[MSG_TX_1_SIZE];

    buffer[0] = MSG_TX_1_HEADER & 0xFF;
    buffer[1] = MSG_TX_1_HEADER >> 8;

#pragma HLS loop unroll
    for (int i = 2; i < MSG_TX_1_SIZE - 2; ++i)
        buffer[i] = msg_tx[i - 2];

    uint16_t crc = checksum_crc16_ccitt(buffer, MSG_TX_1_SIZE - 2);
    buffer[MSG_TX_1_SIZE - 2] = (crc >> 8) & 0xFF;
    buffer[MSG_TX_1_SIZE - 1] = crc & 0xFF;
    
    for (int i = 0; i < MSG_TX_1_SIZE; ++i)
        data_out.write(buffer[i]);
}

int main(int argc, char** argv) {
    // Test the functions here
    return 0;
}