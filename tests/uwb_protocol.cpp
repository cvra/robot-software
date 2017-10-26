#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cstring>
#include "uwb_protocol.h"

TEST_GROUP(MACLayerTestCase)
{
};

TEST(MACLayerTestCase, EncodeFrame)
{
    uint8_t frame[128];
    uint16_t src = 0xbeef, dst = 0xcafe, pan_id = 0xfeeb;
    uint8_t seq_num = 10;

    // Poison the frame with 0xff, then puts in some data
    memset(frame, 0xca, sizeof(frame));

    frame[0] = 0xca;
    frame[1] = 0xfe;

    auto size = uwb_mac_encapsulate_frame(pan_id, src, dst, seq_num, frame, 2);

    // Check that frame control is correct (data, 16 bit address, pan id // compression)
    BYTES_EQUAL(0x41, frame[0]);
    BYTES_EQUAL(0x88, frame[1]);

    // Check the sequence number
    BYTES_EQUAL(10, frame[2]);

    // Check that the PAN id is correct
    BYTES_EQUAL(0xeb, frame[3]);
    BYTES_EQUAL(0xfe, frame[4]);

    // Check that destination MAC address is correct
    BYTES_EQUAL(0xfe, frame[5]);
    BYTES_EQUAL(0xca, frame[6]);

    // Check that source MAC address is correct
    BYTES_EQUAL(0xef, frame[7]);
    BYTES_EQUAL(0xbe, frame[8]);

    // Check that the frame content was modified
    CHECK_EQUAL(0xca, frame[9]);
    CHECK_EQUAL(0xfe, frame[10]);

    int hdr_size = 9, checksum_size = 2;
    CHECK_EQUAL(hdr_size + 2 + checksum_size, size);

    // Check that the placeholders for the checksum were zerod
    CHECK_EQUAL(0x00, frame[11]);
    CHECK_EQUAL(0x00, frame[12]);
}

TEST(MACLayerTestCase, CanDecodeFrame)
{
    const char msg[] = "hello";
    uint8_t frame[128];
    uint16_t src = 0xbeef, dst = 0xcafe, pan_id = 0xfeeb;
    uint16_t decoded_src, decoded_dst, decoded_pan_id;

    uint8_t seq = 23, decoded_seq;

    strcpy((char *)frame, msg);

    auto size = uwb_mac_encapsulate_frame(pan_id, src, dst, seq, frame, sizeof(msg));
    size = uwb_mac_decapsulate_frame(&decoded_pan_id,
                                     &decoded_src,
                                     &decoded_dst,
                                     &decoded_seq,
                                     frame,
                                     size);

    CHECK_EQUAL(src, decoded_src);
    CHECK_EQUAL(dst, decoded_dst);
    CHECK_EQUAL(pan_id, decoded_pan_id);
    CHECK_EQUAL(seq, decoded_seq);

    STRCMP_EQUAL(msg, (char *)frame);
    CHECK_EQUAL(sizeof(msg), size);
}

TEST_GROUP(RangingProtocol)
{
    // So we want to feed the frame in the state
    //
    // Then we should check if we need to send something
    //
    // And also check if we found a ranging solution
    //
    // uwb_ranging_handle_frame(frame);
    // if (uwb_found_ranging_solution()) {
    //     // get the solution
    // } else if (uwb_should_output()) {
    //     uwb_output(output_frame, output_timestamp)
    // }

    void write_40bit_uint(uint64_t val, uint8_t *bytes)
    {
        bytes[0] = (val >> 32) & 0xff;
        bytes[1] = (val >> 24) & 0xff;
        bytes[2] = (val >> 16) & 0xff;
        bytes[3] = (val >> 8) & 0xff;
        bytes[4] = (val >> 0) & 0xff;
    }

    uint64_t read_40bit_uint(uint8_t *bytes)
    {
        uint64_t res = 0;
        for (int i = 0; i < 5; i++) {
            res = (res << 8) | bytes[i];
        }
        return res;
    }

    uwb_protocol_handler_t handler;
    uwb_protocol_handler_t tx_handler;

    // Various buffer used in tests
    uint8_t advertisement_frame[64];
    uint8_t reply_frame[64];
    uint8_t final_frame[64];

    void setup(void)
    {
        uwb_protocol_handler_init(&handler);
        uwb_protocol_handler_init(&tx_handler);

        // put some poison values
        handler.pan_id = 0xaabb;
        handler.address = 0xccdd;
        tx_handler.pan_id = handler.pan_id;
        tx_handler.address = 0xeeff;

        memset(advertisement_frame, 0, sizeof(advertisement_frame));
        memset(reply_frame, 0, sizeof(reply_frame));
        memset(final_frame, 0, sizeof(final_frame));
    }
};

TEST(RangingProtocol, Helpers)
{
    uint8_t frame[5];
    uint8_t expected_frame[] = {0x0f, 0xde, 0xca, 0xca, 0xfe};
    write_40bit_uint(0xfdecacafe, frame);
    MEMCMP_EQUAL(expected_frame, frame, 5);

    auto res = read_40bit_uint(frame);
    LONGS_EQUAL(0xfdecacafe, res);
}

extern "C"
uint64_t uwb_timestamp_get(void)
{
    return mock().actualCall("uwb_timestamp_get").returnIntValue();
}

extern "C"
void uwb_transmit_frame(uint64_t tx_timestamp, uint8_t *frame, size_t frame_size)
{
    mock().actualCall("uwb_transmit_frame").withMemoryBufferParameter("frame", frame, frame_size);
}

TEST(RangingProtocol, PrepareAdvertisementFrame)
{
    uint16_t src, dst, pan_id;
    uint8_t seq;
    uint64_t tx_ts = 1600;

    auto size =
        uwb_protocol_prepare_measurement_advertisement(&handler, tx_ts, advertisement_frame);

    size = uwb_mac_decapsulate_frame(&pan_id, &src, &dst, &seq, advertisement_frame, size);

    CHECK_EQUAL(5, size);
    CHECK_EQUAL(handler.pan_id, pan_id);
    CHECK_EQUAL(handler.address, src);
    CHECK_EQUAL(MAC_802_15_4_BROADCAST_ADDR, dst);
    CHECK_EQUAL(0, seq);

    CHECK_EQUAL(tx_ts, read_40bit_uint(advertisement_frame));
}


TEST(RangingProtocol, SendAdvertisementFrame)
{
    const uint64_t ts = 1600;
    size_t frame_size;

    frame_size = uwb_protocol_prepare_measurement_advertisement(&handler, ts, advertisement_frame);

    mock().expectOneCall("uwb_timestamp_get").andReturnValue(600);
    mock().expectOneCall("uwb_transmit_frame")
    .withMemoryBufferParameter("frame", advertisement_frame, frame_size);

    uint8_t buffer[128];
    uwb_send_measurement_advertisement(&handler, buffer);
}

TEST(RangingProtocol, SendMeasurementReply)
{
    uint64_t advertisement_tx_ts = 600;
    uint64_t advertisement_rx_ts = 1400;
    uint64_t reply_tx_ts = 2400;

    // Prepare the frame to feed in the protocol handler
    auto rx_size = uwb_protocol_prepare_measurement_advertisement(&tx_handler,
                                                                  advertisement_tx_ts,
                                                                  advertisement_frame);

    // Expected frame contains the 3 timestamps (see protocol description in report)
    write_40bit_uint(advertisement_tx_ts, &reply_frame[0]);
    write_40bit_uint(advertisement_rx_ts, &reply_frame[5]);
    write_40bit_uint(reply_tx_ts, &reply_frame[10]);
    size_t reply_size = 15;

    // Prepares the reply frame. It goes from the tag (handler.address) to the
    // anchor (tx_handler.address). Its sequence number must be 1
    reply_size = uwb_mac_encapsulate_frame(tx_handler.pan_id,
                                           handler.address,
                                           tx_handler.address,
                                           1,      // sequence number
                                           reply_frame,
                                           reply_size);

    // Feed the received frame in the processor, expecting a response to get written
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame",
                                                                         reply_frame,
                                                                         reply_size);
    uwb_process_incoming_frame(&handler, advertisement_frame, rx_size, advertisement_rx_ts);

    // Must be called before the stack allocated memory buffers get freed
    mock().checkExpectations();
}

TEST(RangingProtocol, ReplyIsFollowedByFinalMessage)
{
    uint64_t advertisement_tx_ts = 600;
    uint64_t advertisement_rx_ts = 1400;
    uint64_t reply_tx_ts = 2400;
    uint64_t reply_rx_ts = 2600;
    uint64_t final_tx_ts = 3600;

    // Prepare a measurement reply frame
    write_40bit_uint(advertisement_tx_ts, &reply_frame[0]);
    write_40bit_uint(advertisement_rx_ts, &reply_frame[5]);
    write_40bit_uint(reply_tx_ts, &reply_frame[10]);
    size_t reply_size = 15;
    reply_size = uwb_mac_encapsulate_frame(tx_handler.pan_id,
                                           tx_handler.address,
                                           handler.address,
                                           1,      // sequence number
                                           reply_frame,
                                           reply_size);

    // The finalization frame should contain all the five timestamps (see doc).
    write_40bit_uint(advertisement_tx_ts, &final_frame[0]);
    write_40bit_uint(advertisement_rx_ts, &final_frame[5]);
    write_40bit_uint(reply_tx_ts, &final_frame[10]);
    write_40bit_uint(reply_rx_ts, &final_frame[15]);
    write_40bit_uint(final_tx_ts, &final_frame[20]);
    size_t final_size = 25;
    final_size = uwb_mac_encapsulate_frame(tx_handler.pan_id,
                                           handler.address,
                                           tx_handler.address,
                                           2,      // sequence number
                                           final_frame,
                                           final_size);

    // Feed the received frame in the processor, expecting a response to get written
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame",
                                                                         final_frame,
                                                                         final_size);
    uwb_process_incoming_frame(&handler, reply_frame, reply_size, reply_rx_ts);
    mock().checkExpectations();
}

static void ranging_cb(uint16_t anchor_addr, uint64_t propagation_time)
{
    mock().actualCall("ranging_cb").withIntParameter("anchor", anchor_addr).withIntParameter("time",
                                                                                             propagation_time);
}

TEST(RangingProtocol, RangingSolutionIsComputedFinally)
{
    uint64_t advertisement_tx_ts = 600;
    uint64_t advertisement_rx_ts = 1450;
    uint64_t reply_tx_ts = 2450;
    uint64_t reply_rx_ts = 3650;
    uint64_t final_tx_ts = 4650;
    uint64_t final_rx_ts = 5500;

    // Prepare the measurement finalization
    write_40bit_uint(advertisement_tx_ts, &final_frame[0]);
    write_40bit_uint(advertisement_rx_ts, &final_frame[5]);
    write_40bit_uint(reply_tx_ts, &final_frame[10]);
    write_40bit_uint(reply_rx_ts, &final_frame[15]);
    write_40bit_uint(final_tx_ts, &final_frame[20]);
    size_t final_size = 25;
    final_size = uwb_mac_encapsulate_frame(tx_handler.pan_id,
                                           tx_handler.address,
                                           handler.address,
                                           2,      // sequence number
                                           final_frame,
                                           final_size);

    // The callback should be triggered when a solution is found
    handler.ranging_found_cb = ranging_cb;
    mock().expectOneCall("ranging_cb").withIntParameter("anchor", tx_handler.address)
    .withIntParameter("time", 1025);
    uwb_process_incoming_frame(&handler, final_frame, final_size, final_rx_ts);
}

TEST(RangingProtocol, BadPANIDs)
{
    // Change the PAN IDs so that they don't match anymore
    tx_handler.pan_id = 0xbabc;
    handler.pan_id = 0xcafe;

    uint64_t ts = 600;

    auto rx_size = uwb_protocol_prepare_measurement_advertisement(&tx_handler,
                                                                  ts,
                                                                  advertisement_frame);

    // No frame should be sent because PAN IDs dont match
    uwb_process_incoming_frame(&handler, advertisement_frame, rx_size, ts);
}

TEST(RangingProtocol, BadDstAddress)
{
    uint64_t ts = 600;

    auto rx_size = uwb_protocol_prepare_measurement_advertisement(&tx_handler,
                                                                  ts,
                                                                  advertisement_frame);

    // Change the destination address from broadcast to the wrong unicast
    advertisement_frame[5] = advertisement_frame[6] = 0x00;

    // No frame should be sent because PAN IDs dont match
    uwb_process_incoming_frame(&handler, advertisement_frame, rx_size, ts);
}
