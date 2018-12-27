#include "CppUTest/TestHarness.h"
#include "cmp_mem_access.h"
#include "cmp/cmp.h"

TEST_GROUP(CMPMemAccess)
{
    cmp_mem_access_t ma;
    cmp_ctx_t cmp;
};

TEST(CMPMemAccess, Read)
{
    static char testbuf[] = {'\x93', 0x01, 0x02, 0x03}; // [1, 2, 3]
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    bool err = false;
    uint32_t arr_size;
    err = !cmp_read_array(&cmp, &arr_size);
    CHECK(!err);
    CHECK_EQUAL(3, arr_size);
    int32_t i;
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(1, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(2, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(3, i);
}

TEST(CMPMemAccess, ReadOverBufferEnd)
{
    static char testbuf[] = {'\x93', 0x01, 0x02, 0x03}; // [1, 2, 3]
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf)-1);
    bool err = false;
    uint32_t arr_size;
    err = !cmp_read_array(&cmp, &arr_size);
    CHECK(!err);
    CHECK_EQUAL(3, arr_size);
    int32_t i;
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(1, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(!err);
    CHECK_EQUAL(2, i);
    err = !cmp_read_int(&cmp, &i);
    CHECK(err); // error must be set (end of buffer)
}

TEST(CMPMemAccess, Write)
{
    static char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    bool err = false;
    err = !cmp_write_array(&cmp, 3);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 1);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 2);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 3);
    CHECK(!err);
    CHECK_EQUAL((char)'\x93', testbuf[0]);
    CHECK_EQUAL(1, testbuf[1]);
    CHECK_EQUAL(2, testbuf[2]);
    CHECK_EQUAL(3, testbuf[3]);
}

TEST(CMPMemAccess, WriteOverBufferEnd)
{
    static char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf) - 1);
    bool err = false;
    err = !cmp_write_array(&cmp, 3);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 1);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 2);
    CHECK(!err);
    err = !cmp_write_sint(&cmp, 3);
    CHECK(err); // error must be set (end of buffer)
}

TEST(CMPMemAccess, WriteROFails)
{
    static const char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_ro_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    bool err = false;
    err = !cmp_write_array(&cmp, 3);
    CHECK(err);
}

TEST(CMPMemAccess, SetPos)
{
    static char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    cmp_mem_access_set_pos(&ma, 1);
    CHECK_EQUAL(1, ma.index);
}

TEST(CMPMemAccess, GetPtr)
{
    static char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    POINTERS_EQUAL(&testbuf[1], cmp_mem_access_get_ptr_at_pos(&ma, 1));
}

TEST(CMPMemAccess, PosValid)
{
    static char testbuf[4] = {0, 0, 0, 0};
    cmp_mem_access_init(&cmp, &ma, &testbuf, sizeof(testbuf));
    CHECK_TRUE(cmp_mem_access_pos_is_valid(&ma, 0))
    CHECK_TRUE(cmp_mem_access_pos_is_valid(&ma, 3))
    CHECK_FALSE(cmp_mem_access_pos_is_valid(&ma, 4))
}
