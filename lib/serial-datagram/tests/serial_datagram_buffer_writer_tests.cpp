#include "CppUTest/TestHarness.h"
#include "../serial_datagram.h"
#include "../serial_datagram_buffer_writer.h"
#include <cstring>

TEST_GROUP(SerialDatagramBuffferWriterTestCase)
{
    uint8_t buffer[20];
    serial_datagram_buffer_writer_t writer;

    void setup(void)
    {
        memset(&buffer, 0, sizeof buffer);
        serial_datagram_buffer_writer_init(&writer, buffer, sizeof buffer);
    }
};

TEST(SerialDatagramBuffferWriterTestCase, CanInit)
{
    POINTERS_EQUAL(buffer, writer.buffer);
    CHECK_EQUAL(sizeof buffer, writer.buffer_size);
    CHECK_EQUAL(0, writer.write_index);
}

TEST(SerialDatagramBuffferWriterTestCase, CanWrite)
{
    serial_datagram_buffer_writer_cb((void *)&writer, (const void *)"hello", 5);
    serial_datagram_buffer_writer_cb((void *)&writer, (const void *)"hello", 5);

    STRCMP_EQUAL("hellohello", (char *)buffer);
}

TEST(SerialDatagramBuffferWriterTestCase, DoesntOverflow)
{
    uint8_t smallbuf[3];

    memset(smallbuf, 0x55, sizeof smallbuf);

    serial_datagram_buffer_writer_init(&writer, smallbuf, sizeof smallbuf - 1);

    serial_datagram_buffer_writer_cb((void *)&writer, (void *)"hello", 5);

    BYTES_EQUAL('h', smallbuf[0]);
    BYTES_EQUAL('e', smallbuf[1]);
    BYTES_EQUAL(0x55, smallbuf[2]);
}

TEST(SerialDatagramBuffferWriterTestCase, CanWrapBuffer)
{
    uint8_t data[] = "hello";
    size_t ret;

    ret = serial_datagram_buffer_wrap(data, sizeof data, buffer, sizeof buffer);

    BYTES_EQUAL(0xc0, buffer[10]);
    CHECK_EQUAL(sizeof data + sizeof(uint32_t) + 1, ret); // data + crc32 + end
}


