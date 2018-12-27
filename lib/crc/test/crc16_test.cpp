#include "CppUTest/TestHarness.h"
#include "../crc16.h"

TEST_GROUP(CRC16TestGroup)
{

};

TEST(CRC16TestGroup, CorrectCRC)
{
    const char *data = "123456789";
    CHECK_EQUAL(0xBB3D, crc16(0, data, 9));
}

TEST(CRC16TestGroup, NonConstantOutput)
{
    CHECK(crc16(0, "foo", 3) != crc16(0, "bar", 3));
}


TEST(CRC16TestGroup, TestChaining)
{
    uint16_t crc = crc16(0, "1234", 4);
    crc = crc16(crc, "56789", 5);
    CHECK_EQUAL(0xBB3D, crc);
}

TEST(CRC16TestGroup, TestZero)
{
    uint16_t crc = crc16(0, "cvra", 4);
    uint8_t crc_buf[] = {(uint8_t)crc, (uint8_t)(crc>>8)};
    crc = crc16(crc, crc_buf, sizeof(crc_buf));
    CHECK_EQUAL(0, crc);
}
