#include "CppUTest/TestHarness.h"
#include "../crc32.h"

TEST_GROUP(CRC32TestGroup)
{

};

TEST(CRC32TestGroup, CorrectCRC)
{
    const char *data = "123456789";
    CHECK_EQUAL(0xcbf43926, crc32(0, data, 9));
}

TEST(CRC32TestGroup, NonConstantOutput)
{
    CHECK(crc32(0, "foo", 3) != crc32(0, "bar", 3));
}

TEST(CRC32TestGroup, TestChaining)
{
    uint32_t crc = crc32(0, "1234", 4);
    crc = crc32(crc, "56789", 5);
    CHECK_EQUAL(0xcbf43926, crc);
}

TEST(CRC32TestGroup, TestZero)
{
    uint32_t crc = crc32(0, "cvra", 4);
    // crc32() returns an inverted remainder
    uint8_t crc_buf[] = {
        (uint8_t)~crc,
        (uint8_t)(~crc>>8),
        (uint8_t)(~crc>>16),
        (uint8_t)(~crc>>24)
    };
    crc = crc32(crc, crc_buf, sizeof(crc_buf));
    CHECK_EQUAL(0, ~crc);
}
