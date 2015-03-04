#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <../service_call.h>
#include <cstring>

TEST_GROUP(ServiceCallTestGroup)
{
    uint8_t buffer[1024];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;

    void setup(void)
    {
        memset(buffer, 0, sizeof buffer);
    }
};

TEST(ServiceCallTestGroup, CanEncodeServiceCall)
{
    char method_name[1024];
    bool result;
    uint32_t size = sizeof method_name;
    uint32_t map_size;

    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 12);

    cmp_mem_access_set_pos(&mem, 0);
    result = cmp_read_str(&ctx, method_name, &size);

    CHECK_TRUE(result);
    STRCMP_EQUAL(method_name, "foo");

    result = cmp_read_map(&ctx, &map_size);
    CHECK_TRUE(result);
    CHECK_EQUAL(12, map_size);
}
