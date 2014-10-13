#include "CppUTest/TestHarness.h"
#include "../crc32.h"

TEST_GROUP(CRC32TestGroup)
{

};

TEST(CRC32TestGroup, SingleTest)
{
    const char *data = "123456789";
    uint32_t init = 0;
    uint32_t crc = crc32(init, data, strlen(data));
    uint32_t expect = 0xcbf43926;
    CHECK_EQUAL(expect, crc);
}
