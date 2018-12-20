#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <cstring>
#include "uwb_protocol.h"

#define UWB_TX_DELAY 1000

TEST_GROUP (MACLayerTestCase) {
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

    auto hdr_size = 9u, checksum_size = 2u;
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

    strcpy((char*)frame, msg);

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

    STRCMP_EQUAL(msg, (char*)frame);
    CHECK_EQUAL(sizeof(msg), size);
}

TEST_GROUP (RangingProtocol) {
    void write_40bit_uint(uint64_t val, uint8_t * bytes)
    {
        bytes[0] = (val >> 32) & 0xff;
        bytes[1] = (val >> 24) & 0xff;
        bytes[2] = (val >> 16) & 0xff;
        bytes[3] = (val >> 8) & 0xff;
        bytes[4] = (val >> 0) & 0xff;
    }

    uint64_t read_40bit_uint(uint8_t * bytes)
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

extern "C" uint64_t uwb_timestamp_get(void)
{
    return mock().actualCall("uwb_timestamp_get").returnIntValue();
}

extern "C" void uwb_transmit_frame(uint64_t tx_timestamp, uint8_t* frame, size_t frame_size)
{
    mock().actualCall("uwb_transmit_frame").withUnsignedLongIntParameter("timestamp", tx_timestamp).withMemoryBufferParameter("frame", frame, frame_size);
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

    // The message should be sent when the lower 9 bits are zero
    uint64_t tx_ts = ts + (1000 * 65536ULL);
    tx_ts &= ~(0x1FFULL);

    frame_size =
        uwb_protocol_prepare_measurement_advertisement(&handler, tx_ts, advertisement_frame);

    mock().expectOneCall("uwb_timestamp_get").andReturnValue((int)ts);
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame", advertisement_frame, frame_size).withUnsignedLongIntParameter("timestamp", tx_ts);

    uint8_t buffer[128];
    uwb_send_measurement_advertisement(&handler, buffer);
}

TEST(RangingProtocol, SendMeasurementReply)
{
    uint64_t advertisement_tx_ts = 600;
    uint64_t advertisement_rx_ts = 1400;
    uint64_t reply_tx_ts = 2400;

    // The final message should be sent when the lower 9 bits are zero
    reply_tx_ts = advertisement_rx_ts + (UWB_TX_DELAY * 65536ULL);
    reply_tx_ts &= ~(0x1FFULL);

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
                                           1, // sequence number
                                           reply_frame,
                                           reply_size);

    // Feed the received frame in the processor, expecting a response to get written
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame", reply_frame, reply_size).withUnsignedLongIntParameter("timestamp", reply_tx_ts);

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
    uint64_t final_tx_ts;

    // The final message should be sent at a time where the lower 9 bits
    // are zero
    final_tx_ts = reply_rx_ts + (UWB_TX_DELAY * 65536ULL);
    final_tx_ts &= ~(0x1FFULL);

    // Prepare a measurement reply frame
    write_40bit_uint(advertisement_tx_ts, &reply_frame[0]);
    write_40bit_uint(advertisement_rx_ts, &reply_frame[5]);
    write_40bit_uint(reply_tx_ts, &reply_frame[10]);
    size_t reply_size = 15;
    reply_size = uwb_mac_encapsulate_frame(tx_handler.pan_id,
                                           tx_handler.address,
                                           handler.address,
                                           1, // sequence number
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
                                           2, // sequence number
                                           final_frame,
                                           final_size);

    // Feed the received frame in the processor, expecting a response to get written
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame", final_frame, final_size).withUnsignedLongIntParameter("timestamp", final_tx_ts);

    uwb_process_incoming_frame(&handler, reply_frame, reply_size, reply_rx_ts);
    mock().checkExpectations();
}

static void ranging_cb(uint16_t anchor_addr, uint64_t propagation_time)
{
    mock().actualCall("ranging_cb").withIntParameter("anchor", anchor_addr).withIntParameter("time", propagation_time);
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
                                           2, // sequence number
                                           final_frame,
                                           final_size);

    // The callback should be triggered when a solution is found
    handler.ranging_found_cb = ranging_cb;
    mock().expectOneCall("ranging_cb").withIntParameter("anchor", tx_handler.address).withIntParameter("time", 1025);
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

TEST(RangingProtocol, DoNotReplyToadvertisementWhenWeAreAnAnchor)
{
    uint64_t ts = 600;
    auto rx_size = uwb_protocol_prepare_measurement_advertisement(&tx_handler,
                                                                  ts,
                                                                  advertisement_frame);

    // Configure the receiving beacon as an anchor
    handler.is_anchor = true;

    // No reply should be sent, hence no mock() call.
    uwb_process_incoming_frame(&handler, advertisement_frame, rx_size, ts);
}

// TODO: If we are an anchor we should not answer to advertisement
#define UINT40_MAX ((1UL << 40) - 1)
TEST(RangingProtocol, Overflow)
{
    uint64_t advertisement_tx_ts = UINT40_MAX - 2000;
    uint64_t advertisement_rx_ts = UINT40_MAX - 1150;
    uint64_t reply_tx_ts = UINT40_MAX - 150;
    uint64_t reply_rx_ts = 1050;
    uint64_t final_tx_ts = 2050;
    uint64_t final_rx_ts = 2900;

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
                                           2, // sequence number
                                           final_frame,
                                           final_size);

    // The callback should be triggered when a solution is found
    handler.ranging_found_cb = ranging_cb;
    mock().expectOneCall("ranging_cb").withIntParameter("anchor", tx_handler.address).withIntParameter("time", 1025);
    uwb_process_incoming_frame(&handler, final_frame, final_size, final_rx_ts);
}

TEST_GROUP (AnchorPositionBroadcast) {
    uwb_protocol_handler_t handler;
    uint8_t frame[128];
    uint16_t src, dst, pan_id;
    uint8_t seq;
    size_t size;

    const float x = 10, y = 20, z = 30;
    float rx, ry, rz;

    void setup(void)
    {
        uwb_protocol_handler_init(&handler);
        handler.address = 1234;
        handler.pan_id = 4321;
        memset(frame, 0, sizeof(frame));

        // Creates the anchor position message
        size = uwb_protocol_prepare_anchor_position(&handler, x, y, z, frame);

        // Extracts the payload
        size = uwb_mac_decapsulate_frame(&pan_id, &src, &dst, &seq, frame, size);
    }
};

TEST(AnchorPositionBroadcast, PositionIsEncodedCorrectly)
{
    // We expect 3 floats (x, y, z) -> 12 bytes
    CHECK_EQUAL(12, size);

    // Extract the floats from the message
    memcpy(&rx, &frame[0], sizeof(float));
    memcpy(&ry, &frame[4], sizeof(float));
    memcpy(&rz, &frame[8], sizeof(float));

    CHECK_EQUAL(x, rx);
    CHECK_EQUAL(y, ry);
    CHECK_EQUAL(z, rz);
}

TEST(AnchorPositionBroadcast, SourceIsCorrect)
{
    CHECK_EQUAL(src, handler.address);
}

TEST(AnchorPositionBroadcast, MessageIsSentToBroadcast)
{
    CHECK_EQUAL(0xffff, dst);
}

TEST(AnchorPositionBroadcast, MessageIsOnTheCorrectPAN)
{
    CHECK_EQUAL(handler.pan_id, pan_id);
}

TEST(AnchorPositionBroadcast, SequenceNumberIsCorrect)
{
    // 3 is anchor position
    CHECK_EQUAL(3, seq);
}

TEST(AnchorPositionBroadcast, SendAnchorPosition)
{
    // Creates the anchor position message
    size = uwb_protocol_prepare_anchor_position(&handler, x, y, z, frame);

    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame", frame, size).withUnsignedLongIntParameter("timestamp", UWB_TX_TIMESTAMP_IMMEDIATE);

    uint8_t buffer[64];
    uwb_send_anchor_position(&handler, x, y, z, buffer);
}

void anchor_position_received_cb(uint16_t anchor_addr, float x, float y, float z)
{
    mock().actualCall("anchor_position_cb").withIntParameter("anchor_addr", anchor_addr).withParameter("x", x).withParameter("y", y).withParameter("z", z);
}

TEST(AnchorPositionBroadcast, ReceiveAnchorPosition)
{
    // Creates the anchor position message
    size = uwb_protocol_prepare_anchor_position(&handler, x, y, z, frame);

    mock().expectOneCall("anchor_position_cb").withIntParameter("anchor_addr", handler.address).withParameter("x", x).withParameter("y", y).withParameter("z", z);
    handler.anchor_position_received_cb = anchor_position_received_cb;

    // Pass 0 as the RX timestamp is not used
    uwb_process_incoming_frame(&handler, frame, size, 0);
}

TEST_GROUP (TagPositionBroadcast) {
    uwb_protocol_handler_t handler;
    uint8_t frame[128];
    uint16_t src, dst, pan_id;
    uint8_t seq;
    size_t size;

    const float x = 10, y = 20;

    void setup(void)
    {
        uwb_protocol_handler_init(&handler);
        handler.address = 1234;
        handler.pan_id = 4321;
        memset(frame, 0, sizeof(frame));

        // Creates the anchor position message
        size = uwb_protocol_prepare_tag_position(&handler, x, y, frame);

        // Extracts the payload
        size = uwb_mac_decapsulate_frame(&pan_id, &src, &dst, &seq, frame, size);
    }
};

TEST(TagPositionBroadcast, AddressingIsCorrect)
{
    CHECK_EQUAL(1234, src);
    CHECK_EQUAL(4321, pan_id);

    // Tag position is broadcast
    CHECK_EQUAL(0xffff, dst);
}

TEST(TagPositionBroadcast, MessageContentIsCorrect)
{
    CHECK_EQUAL(4, seq);
    CHECK_EQUAL(8, size);

    float* buffer = (float*)frame;

    CHECK_EQUAL(x, buffer[0]);
    CHECK_EQUAL(y, buffer[1]);
}

TEST(TagPositionBroadcast, SendTagPosition)
{
    size = uwb_protocol_prepare_tag_position(&handler, x, y, frame);
    mock().expectOneCall("uwb_transmit_frame").withMemoryBufferParameter("frame", frame, size).withUnsignedLongIntParameter("timestamp", UWB_TX_TIMESTAMP_IMMEDIATE);

    uint8_t buffer[64];
    uwb_send_tag_position(&handler, x, y, buffer);
}

void tag_position_received_cb(uint16_t tag_addr, float x, float y)
{
    mock().actualCall("tag_position_cb").withIntParameter("tag_addr", tag_addr).withParameter("x", x).withParameter("y", y);
}

TEST(TagPositionBroadcast, ReceiveTagPosition)
{
    // Creates the tag position message
    size = uwb_protocol_prepare_tag_position(&handler, x, y, frame);

    mock().expectOneCall("tag_position_cb").withIntParameter("tag_addr", handler.address).withParameter("x", x).withParameter("y", y);
    handler.tag_position_received_cb = tag_position_received_cb;

    // Pass 0 as the RX timestamp is not used
    uwb_process_incoming_frame(&handler, frame, size, 1);
}
