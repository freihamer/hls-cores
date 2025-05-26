#include "rxdeframe.hpp"
#include "checksum.hpp"

enum class ReceiverState {
    Idle,
    Starting,
    Receiving,
    Completed
};

class NavMsgReceiver {
public:
    NavMsgReceiver(uint16_t header, uint8_t size) :
        state(ReceiverState::Idle),
        counter(0),
        buffer{0},
        header(header),
        size(size) {}

    void reset() {
        counter = 0;
        state = ReceiverState::Idle;
    }

    ReceiverState get_state() const {
        return state;
    }

    const uint8_t (&get_buffer() const)[256] {
        return buffer;
    }

    uint8_t get_counter() const {
        return counter;
    }

    ReceiverState process(uint8_t byte_rx) {
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
                    state = ReceiverState::Completed;
                }
                break;

            case ReceiverState::Completed:
                break;
        }

        return state;
    }

private:

    ReceiverState state;
    uint8_t counter;
    uint8_t buffer[256];
    uint16_t header;
    uint8_t size;
    
};


void urano_nav_rx(
    hls::FIFO<uint8_t> &data_in,
    uint64_t* msg_1_timestamp,
    uint64_t* msg_2_timestamp,                  
    uint8_t* msg_1_payload,
    uint8_t* msg_2_payload,
    NavMsgStats* msg_1_stats,
    NavMsgStats* msg_2_stats
) {
#pragma HLS function top
#pragma HLS interface control type(simple)
#pragma HLS interface argument(msg_1_timestamp) type(axi_target) num_elements(1)
#pragma HLS interface argument(msg_2_timestamp) type(axi_target) num_elements(1)
#pragma HLS interface argument(msg_1_payload) type(axi_target) num_elements(RX_MSG_1_SIZE - 4)
#pragma HLS interface argument(msg_2_payload) type(axi_target) num_elements(RX_MSG_2_SIZE - 4)
#pragma HLS interface argument(msg_1_stats) type(axi_target) num_elements(1)
#pragma HLS interface argument(msg_2_stats) type(axi_target) num_elements(1)

    static uint64_t clock_cycles = 0U;
    static uint16_t gap_tracker = 0U; // The number of clock clock_cycles spent waiting for the next byte_rx of a sequence

    static NavMsgReceiver msg_1_receiver(RX_MSG_1_HEADER, RX_MSG_1_SIZE);
    static NavMsgReceiver msg_2_receiver(RX_MSG_2_HEADER, RX_MSG_2_SIZE);

    
    clock_cycles++;

    if (data_in.empty()) {
        if (msg_1_receiver.get_state() != ReceiverState::Idle && ++gap_tracker >= 5000U) {
            /* The receiver is expecting the next byte, but the gap tracker has reached the threshold, indicating a timeout.
             * The gap tracker threshold is set to 5000 clock cycles, which corresponds to 100us at a 50MHz clock frequency.
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
            uint16_t expected = (buffer[RX_MSG_2_SIZE - 2] << 8) | buffer[RX_MSG_2_SIZE - 1];
            uint16_t computed = checksum(buffer, RX_MSG_2_SIZE - 2);
            if (expected == computed) {
#pragma HLS loop unroll
                for (uint8_t i = 2; i < RX_MSG_2_SIZE - 2; ++i) {
                    msg_2_payload[i - 2] = buffer[i];
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
        uint16_t expected = (buffer[RX_MSG_1_SIZE - 2] << 8) | buffer[RX_MSG_1_SIZE - 1];
        uint16_t computed = checksum(buffer, RX_MSG_1_SIZE - 2);
        if (expected == computed) {
#pragma HLS loop unroll
            for (uint8_t i = 2; i < RX_MSG_1_SIZE - 2; ++i) {
                msg_1_payload[i - 2] = buffer[i];
            }

            *msg_1_timestamp = clock_cycles;
            msg_1_stats->valid++;
        } else {
            msg_1_stats->invalid++;
        } 
    }
}