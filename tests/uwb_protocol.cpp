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

    frame[0] = 0xca;
    frame[1] = 0xfe;

    uwb_mac_encapsulate_frame(pan_id, src, dst, frame, sizeof(frame));

    // Check that frame control is correct (data, 16 bit address, pan id // compression)
    BYTES_EQUAL(0x41, frame[0]);
    BYTES_EQUAL(0x88, frame[1]);

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
}

TEST(MACLayerTestCase, CanDecodeFrame)
{
    const char msg[] = "hello";
    uint8_t frame[128];
    uint16_t src = 0xbeef, dst = 0xcafe, pan_id = 0xfeeb;
    uint16_t decoded_src, decoded_dst, decoded_pan_id;

    strcpy((char *)frame, msg);

    uwb_mac_encapsulate_frame(pan_id, src, dst, frame, sizeof(frame));
    uwb_mac_decapsulate_frame(&decoded_pan_id, &decoded_src, &decoded_dst, frame, sizeof(frame));

    CHECK_EQUAL(src, decoded_src);
    CHECK_EQUAL(dst, decoded_dst);
    CHECK_EQUAL(pan_id, decoded_pan_id);

    STRCMP_EQUAL(msg, (char *)frame);
}
