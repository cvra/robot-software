#include "CppUTest/TestHarness.h"
#include "../serial_datagram.h"


/* CRC checksums can be calculated with python using:
 * binascii.hexlify(struct.pack('>i', binascii.crc32('msg')))
 */

#define END         '\xC0'
#define ESC         '\xDB'
#define ESC_END     '\xDC'
#define ESC_ESC     '\xDD'

char sendbuffer[100];
int send_index;

extern "C" void send_fn(void *arg, const void *p, size_t len)
{
    (void)arg;
    const char *byte = (const char*)p;
    // printf("[");
    while (len-- > 0) {
        // unsigned int c = (unsigned char)*p;
        // printf("0x%x ", c);
        sendbuffer[send_index++] = *byte++;
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
    char d[] = {'\x0a', '\x0b', '\x0c'}; // crc 1894c924
    serial_datagram_send(d, sizeof(d), send_fn, NULL);
    BYTES_EQUAL('\x0a', sendbuffer[0]);
    BYTES_EQUAL('\x0b', sendbuffer[1]);
    BYTES_EQUAL('\x0c', sendbuffer[2]);
    // CRC
    BYTES_EQUAL('\x18', sendbuffer[3]);
    BYTES_EQUAL('\x94', sendbuffer[4]);
    BYTES_EQUAL('\xc9', sendbuffer[5]);
    BYTES_EQUAL('\x24', sendbuffer[6]);
    // STOP
    BYTES_EQUAL('\xC0', sendbuffer[7]);
    CHECK_EQUAL(3+4+1, send_index);
}

TEST(SerialDatagramSendTestGroup, SendEmptyFrame)
{
    char d[] = {}; // crc 00000000
    serial_datagram_send(d, sizeof(d), send_fn, NULL);
    // CRC
    BYTES_EQUAL('\x00', sendbuffer[0]);
    BYTES_EQUAL('\x00', sendbuffer[1]);
    BYTES_EQUAL('\x00', sendbuffer[2]);
    BYTES_EQUAL('\x00', sendbuffer[3]);
    // STOP
    BYTES_EQUAL('\xC0', sendbuffer[4]);
    CHECK_EQUAL(4+1, send_index);
}

TEST(SerialDatagramSendTestGroup, SendFrameEscape)
{
    char d[] = {ESC, '\x0a', END, '\x0b'}; // crc 81ae6a94
    serial_datagram_send(d, sizeof(d), send_fn, NULL);
    BYTES_EQUAL(ESC, sendbuffer[0]);
    BYTES_EQUAL(ESC_ESC, sendbuffer[1]);
    BYTES_EQUAL('\x0a', sendbuffer[2]);
    BYTES_EQUAL(ESC, sendbuffer[3]);
    BYTES_EQUAL(ESC_END, sendbuffer[4]);
    BYTES_EQUAL('\x0b', sendbuffer[5]);
    // CRC
    BYTES_EQUAL('\x81', sendbuffer[6]);
    BYTES_EQUAL('\xae', sendbuffer[7]);
    BYTES_EQUAL('\x6a', sendbuffer[8]);
    BYTES_EQUAL('\x94', sendbuffer[9]);
    // STOP
    BYTES_EQUAL('\xC0', sendbuffer[10]);
    CHECK_EQUAL(2*2+2+4+1, send_index);
}

TEST(SerialDatagramSendTestGroup, SendFrameEscapedCRC)
{
    char d[] = {'s', 'd', 'a', 'f', '\n', 0}; // crc 65e9c03f
    serial_datagram_send(d, sizeof(d), send_fn, NULL);
    BYTES_EQUAL('s', sendbuffer[0]);
    BYTES_EQUAL('d', sendbuffer[1]);
    BYTES_EQUAL('a', sendbuffer[2]);
    BYTES_EQUAL('f', sendbuffer[3]);
    BYTES_EQUAL('\n', sendbuffer[4]);
    BYTES_EQUAL(0, sendbuffer[5]);
    // CRC (escaped 65e9dbdc3fc0)
    BYTES_EQUAL('\x65', sendbuffer[6]);
    BYTES_EQUAL('\xe9', sendbuffer[7]);
    BYTES_EQUAL('\xdb', sendbuffer[8]);
    BYTES_EQUAL('\xdc', sendbuffer[9]);
    BYTES_EQUAL('\x3f', sendbuffer[10]);
    // STOP
    BYTES_EQUAL('\xC0', sendbuffer[11]);
    CHECK_EQUAL(6+5+1, send_index);
}


char *expected_dtgrm;
size_t expected_dtgrm_len;
void *expected_arg;
int rcv_nb_calls;
extern "C" void rcv_cb(const void *dtgrm, size_t len, void *arg)
{
    POINTERS_EQUAL(expected_arg, arg);
    const char *dtgrm_byte = (const char*)dtgrm;
    CHECK_EQUAL(expected_dtgrm_len, len);
    for (int i = 0; i < len; i++) {
        BYTES_EQUAL(expected_dtgrm[i], dtgrm_byte[i]);
    }
    rcv_nb_calls++;
}

TEST_GROUP(SerialDatagramRcvTestGroup)
{
    char buffer[100];
    serial_datagram_rcv_handler_t h;

    void setup(void)
    {
        serial_datagram_rcv_handler_init(&h, buffer, sizeof(buffer), rcv_cb, NULL);
        rcv_nb_calls = 0;
        expected_arg = NULL;
    }
};


TEST(SerialDatagramRcvTestGroup, RcvHandlerInit)
{
    CHECK_EQUAL((void*)buffer, (void*)h.buffer);
    CHECK_EQUAL(sizeof(buffer), h.size);
    CHECK_EQUAL(0, h.write_index);
    CHECK_EQUAL(&rcv_cb, h.callback_fn);
    CHECK_FALSE(h.error_flag);
    CHECK_FALSE(h.esc_flag);
}

TEST(SerialDatagramRcvTestGroup, RcvFrameTooLong)
{
    serial_datagram_rcv_handler_init(&h, buffer, 4, rcv_cb, NULL);
    char frame[] = {'a', '\xe8', '\xb7', '\xbe', '\x43', END}; // crc e8b7be43
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
    char frame[] = {'a', '\xff', '\xff', '\xff', '\xff', END}; // crc e8b7be43
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_CRC_MISMATCH, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvProtocolError)
{
    char frame[] = {ESC, '\xff', END}; // bad escape sequence
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR, ret);
}

TEST(SerialDatagramRcvTestGroup, RcvValidFrame)
{
    char d[] = {'a'};
    expected_dtgrm = d;
    expected_dtgrm_len = 1;

    char frame[] = {'a', '\xe8', '\xb7', '\xbe', '\x43', END}; // crc e8b7be43
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvEmptyFrame)
{
    char d[] = {};
    expected_dtgrm = d;
    expected_dtgrm_len = 0;

    char frame[] = {0, 0, 0, 0, END}; // crc e8b7be43
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvValidEscapedFrame)
{
    char d[] = {'a', ESC, 'b', END}; // crc efad5e3e
    expected_dtgrm = d;
    expected_dtgrm_len = 4;

    char frame[] = {'a', ESC, ESC_ESC, 'b', ESC, ESC_END, '\xef', '\xad', '\x5e', '\x3e', END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvValidEscapedFrameMultiCall)
{
    char d[] = {'a', ESC, 'b', END}; // crc efad5e3e
    expected_dtgrm = d;
    expected_dtgrm_len = 4;

    char frame[] = {'a', ESC, ESC_ESC, 'b', ESC, ESC_END, '\xef', '\xad', '\x5e', '\x3e', END};
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
    char d[] = {'a', 'b'}; // crc 9e83486d
    expected_dtgrm = d;
    expected_dtgrm_len = 2;
    char frame[] = {'a', 'b', '\x9e', '\x83', '\x48', '\x6d', END, 'a', 'b', '\x9e', '\x83', '\x48', '\x6d', END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(2, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvMaxSizeValidFrame)
{
    char d[] = {ESC}; // crc c303e4d1
    expected_dtgrm = d;
    expected_dtgrm_len = 1;

    serial_datagram_rcv_handler_init(&h, buffer, 5, rcv_cb, NULL);
    char frame[] = {ESC, ESC_ESC, '\xc3', '\x03', '\xe4', '\xd1', END};
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}

TEST(SerialDatagramRcvTestGroup, RcvCallbackArg)
{
    serial_datagram_rcv_handler_init(&h, buffer, sizeof(buffer), rcv_cb, (void*)0x1234);
    char d[] = {};
    expected_dtgrm = d;
    expected_dtgrm_len = 0;
    expected_arg = (void*)0x1234;

    char frame[] = {0, 0, 0, 0, END}; // crc e8b7be43
    int ret = serial_datagram_receive(&h, frame, sizeof(frame));
    CHECK_EQUAL(SERIAL_DATAGRAM_RCV_NO_ERROR, ret);
    CHECK_EQUAL(1, rcv_nb_calls);
}
