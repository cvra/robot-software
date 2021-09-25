#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"

TEST_GROUP (SkipObjectTestGroup) {
};

TEST(SkipObjectTestGroup, CanSkipSingleValue)
{
    char buffer[256];
    cmp_mem_access_t mem;
    cmp_ctx_t cmp;
    cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));

    // Write the following object: {8: 16, 6, 12}
    cmp_write_map(&cmp, 2);
    {
        cmp_write_uint(&cmp, 8);
        cmp_write_uint(&cmp, 16);
    }
    {
        cmp_write_uint(&cmp, 6);
        cmp_write_uint(&cmp, 12);
    }

    // Restarts at the beginning of the buffer
    uint32_t val;
    cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));
    cmp_read_map(&cmp, &val);
    CHECK_EQUAL(2, val);
    cmp_read_uint(&cmp, &val);
    CHECK_EQUAL(8, val);
    cmp_skip_object_no_limit(&cmp); // skips the value
    cmp_read_uint(&cmp, &val);
    CHECK_EQUAL(6, val);
}

TEST(SkipObjectTestGroup, CanSkipListsAndDicts)
{
    char buffer[256];
    cmp_mem_access_t mem;
    cmp_ctx_t cmp;
    cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));

    // Write the following object: {8: [1,2,3], 2: {1: 2}, 6: 12}
    cmp_write_map(&cmp, 3);
    {
        cmp_write_uint(&cmp, 8);
        {
            cmp_write_array(&cmp, 3);
            cmp_write_uint(&cmp, 1);
            cmp_write_uint(&cmp, 2);
            cmp_write_uint(&cmp, 3);
        }
    }
    {
        cmp_write_uint(&cmp, 2);
        {
            cmp_write_map(&cmp, 1);
            cmp_write_uint(&cmp, 1);
            cmp_write_uint(&cmp, 2);
        }
    }
    {
        cmp_write_uint(&cmp, 6);
        cmp_write_uint(&cmp, 12);
    }

    // Restarts at the beginning of the buffer
    uint32_t val;
    cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));
    cmp_read_map(&cmp, &val);
    CHECK_EQUAL(3, val);
    cmp_skip_object_no_limit(&cmp); // skips 8
    cmp_skip_object_no_limit(&cmp); // skips [1,2,3]
    cmp_skip_object_no_limit(&cmp); // skips 2
    cmp_skip_object_no_limit(&cmp); // skips {1, 2}
    cmp_read_uint(&cmp, &val);
    CHECK_EQUAL(6, val);
    cmp_read_uint(&cmp, &val);
    CHECK_EQUAL(12, val);
}
