#include "CppUTest/TestHarness.h"
#include "../serial_datagram.h"

#define END         '\xC0'
#define ESC         '\xDB'
#define ESC_END     '\xDC'
#define ESC_ESC     '\xDD'

char sendbuffer[100];
int send_index;

extern "C" void send_fn(const char *p, size_t len)
{
    // printf("[");
    while (len-- > 0) {
        // unsigned int c = (unsigned char)*p;
        // printf("0x%x ", c);
        sendbuffer[send_index++] = *p++;
    }
    // printf("]\n");
}

TEST_GROUP(SerialDatagramSendTestGroup)
{
    void setup(void)
    {
        send_index = 0;
    }
};

TEST(SerialDatagramSendTestGroup, SendFrame)
{
    char d[] = {0x0a, 0x0b, 0x0c};
    serial_datagram_send(d, sizeof(d), send_fn);
    BYTES_EQUAL(0x0a, sendbuffer[0]);
    BYTES_EQUAL(0x0b, sendbuffer[1]);
    BYTES_EQUAL(0x0c, sendbuffer[2]);
    // CRC
    BYTES_EQUAL(0x11, sendbuffer[3]);
    BYTES_EQUAL(0x22, sendbuffer[4]);
    BYTES_EQUAL(0x33, sendbuffer[5]);
    BYTES_EQUAL(0x44, sendbuffer[6]);
    // STOP
    BYTES_EQUAL(0xC0, sendbuffer[7]);
    CHECK_EQUAL(3+4+1, send_index);
}

TEST(SerialDatagramSendTestGroup, SendFrameEscape)
{
    char d[] = {ESC, 0x0a, END, 0x0b};
    serial_datagram_send(d, sizeof(d), send_fn);
    BYTES_EQUAL(ESC, sendbuffer[0]);
    BYTES_EQUAL(ESC_ESC, sendbuffer[1]);
    BYTES_EQUAL(0x0a, sendbuffer[2]);
    BYTES_EQUAL(ESC, sendbuffer[3]);
    BYTES_EQUAL(ESC_END, sendbuffer[4]);
    BYTES_EQUAL(0x0b, sendbuffer[5]);
    // CRC
    BYTES_EQUAL(0x11, sendbuffer[6]);
    BYTES_EQUAL(0x22, sendbuffer[7]);
    BYTES_EQUAL(0x33, sendbuffer[8]);
    BYTES_EQUAL(0x44, sendbuffer[9]);
    // STOP
    BYTES_EQUAL(0xC0, sendbuffer[10]);
    CHECK_EQUAL(2*2+2+4+1, send_index);
}



char *expected_dtgrm;
size_t expected_dtgrm_len;
int rcv_nb_calls;
extern "C" void rcv_cb(const char *dtgrm, size_t len)
{
    CHECK_EQUAL(expected_dtgrm_len, len);
    for (int i = 0; i < len; i++) {
        BYTES_EQUAL(expected_dtgrm[i], dtgrm[i]);
    }
    rcv_nb_calls++;
}

TEST_GROUP(SerialDatagramRcvTestGroup)
{
    char buffer[100];
    serial_datagram_rcv_handler_t h;

    void setup(void)
    {
        serial_datagram_rcv_handler_init(&h, buffer, sizeof(buffer), rcv_cb);
        rcv_nb_calls = 0;
    }
};


TEST(SerialDatagramRcvTestGroup, RcvHandlerInit)
{
    CHECK_EQUAL(buffer, h.buffer);
    CHECK_EQUAL(sizeof(buffer), h.size);
    CHECK_EQUAL(0, h.write_index);
    CHECK_EQUAL(&rcv_cb, h.callback_fn);
    CHECK_FALSE(h.error_flag);
    CHECK_FALSE(h.esc_flag);
}

TEST(SerialDatagramRcvTestGroup, RcvFrameTooLong)
{
    serial_datagram_rcv_handler_init(&h, buffer, 4, rcv_cb);
    char frame[] = {'a', 0x11, 0x22, 0x33, 0x44, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_DATAGRAM_TOO_LONG, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvFrameTooShort)
{
    char frame[] = {END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvCRCError)
{
    char frame[] = {'a', 0xff, 0xff, 0xff, 0xff, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_CRC_MISMATCH, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvProtocolError)
{
    char frame[] = {ESC, 0xff, END}; // bad escape sequence
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvValidFrame)
{
    char d[] = {'a'};
    expected_dtgrm = d;
    expected_dtgrm_len = 1;

    char frame[] = {'a', 0x11, 0x22, 0x33, 0x44, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvValidEscapedFrame)
{
    char d[] = {'a', ESC, 'b', END};
    expected_dtgrm = d;
    expected_dtgrm_len = 4;

    char frame[] = {'a', ESC, ESC_ESC, 'b', ESC, ESC_END, 0x11, 0x22, 0x33, 0x44, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvValidEscapedFrameMultiCall)
{
    char d[] = {'a', ESC, 'b', END};
    expected_dtgrm = d;
    expected_dtgrm_len = 4;

    char frame[] = {'a', ESC, ESC_ESC, 'b', ESC, ESC_END, 0x11, 0x22, 0x33, 0x44, END};
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[0], 2));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[2], 1));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[3], 1));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[4], 1));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[5], 3));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, serial_datagram_receive(&h, &frame[8], 3));
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvMultiValidFrames)
{
    char d[] = {'a', 'b'};
    expected_dtgrm = d;
    expected_dtgrm_len = 2;
    char frame[] = {'a', 'b', 0x11, 0x22, 0x33, 0x44, END, 'a', 'b', 0x11, 0x22, 0x33, 0x44, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(2, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvMaxSizeValidFrame)
{
    char d[] = {ESC};
    expected_dtgrm = d;
    expected_dtgrm_len = 1;

    serial_datagram_rcv_handler_init(&h, buffer, 5, rcv_cb);
    char frame[] = {ESC, ESC_ESC, 0x11, 0x22, 0x33, 0x44, END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}
