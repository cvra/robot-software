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

TEST_GROUP(SerialDatagramTestGroup)
{
    void setup(void)
    {
        send_index = 0;
    }
};

TEST(SerialDatagramTestGroup, SendFrame)
{
    char d[] = {0x0a, 0x0b, 0x0c};
    serial_datagram_send(d, sizeof(d), send_fn);
    BYTES_EQUAL(0x0a, sendbuffer[0]);
    BYTES_EQUAL(0x0b, sendbuffer[1]);
    BYTES_EQUAL(0x0c, sendbuffer[2]);
    // CRC
    BYTES_EQUAL(0x00, sendbuffer[3]);
    BYTES_EQUAL(0x00, sendbuffer[4]);
    BYTES_EQUAL(0x00, sendbuffer[5]);
    BYTES_EQUAL(0x00, sendbuffer[6]);
    // STOP
    BYTES_EQUAL(0xC0, sendbuffer[7]);
    CHECK_EQUAL(3+4+1, send_index);
}

TEST(SerialDatagramTestGroup, SendFrameEscape)
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
    BYTES_EQUAL(0x00, sendbuffer[6]);
    BYTES_EQUAL(0x00, sendbuffer[7]);
    BYTES_EQUAL(0x00, sendbuffer[8]);
    BYTES_EQUAL(0x00, sendbuffer[9]);
    // STOP
    BYTES_EQUAL(0xC0, sendbuffer[10]);
    CHECK_EQUAL(2*2+2+4+1, send_index);
}
