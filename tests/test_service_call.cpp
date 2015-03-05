#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include <../service_call.h>
#include <cstring>

#define LEN(a) (sizeof (a) / sizeof(a[0]))

void foo_cb(int argc, cmp_ctx_t *args_ctx)
{
    mock().actualCall("foo").withIntParameter("argc", argc);
}

void bar_cb(int argc, cmp_ctx_t *args_ctx)
{
    mock().actualCall("bar");
}

/** This callback shows how to read the several arguments. */
void arg_read_cb(int argc, cmp_ctx_t *args_ctx)
{
    bool result;
    int arg;
    char arg_name[64];
    unsigned int arg_len;

    MockActualCall& state = mock().actualCall("arg_read_cb");

    for (int i = 0; i < argc; ++i) {

        // Read the argument name from the map
        arg_len = sizeof arg_name;
        result = cmp_read_str(args_ctx, arg_name, &arg_len);
        CHECK_TRUE(result);

        // Read the argument value
        result = cmp_read_int(args_ctx, &arg);
        CHECK_TRUE(result);

        // Add a parameter to the call list
        state = state.withIntParameter(arg_name, arg);
    }
}

service_call_method callbacks[] = {
    {.name = (char *)"foo", .cb = foo_cb},
    {.name = (char *)"bar", .cb = bar_cb},
    {.name = (char *)"arg_read", .cb = arg_read_cb}
};


TEST_GROUP(ServiceCallTestGroup)
{
    uint8_t buffer[1024];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;

    void setup(void)
    {
        memset(buffer, 0, sizeof buffer);
    }

    void teardown(void)
    {
        mock().checkExpectations();
        mock().clear();
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

TEST(ServiceCallTestGroup, CanProcessServiceCall)
{
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 0);

    mock().expectOneCall("foo").withIntParameter("argc", 0);
    service_call_process(buffer, sizeof buffer, callbacks, LEN(callbacks));
}

TEST(ServiceCallTestGroup, CallsCorrectServiceCall)
{
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "bar", 0);

    mock().expectOneCall("bar");

    service_call_process(buffer, sizeof buffer, callbacks, LEN(callbacks));
}

TEST(ServiceCallTestGroup, PassesArgcCorrectly)
{
    const int argc = 2;

    // Writes the header with two arguments
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "arg_read", argc);

    // Writes the first argument
    cmp_write_str(&ctx, "x", 1);
    cmp_write_uint(&ctx, 12);

    // Writes the second one
    cmp_write_str(&ctx, "y", 1);
    cmp_write_uint(&ctx, 13);

    mock().expectOneCall("arg_read_cb")
          .withIntParameter("x", 12)
          .withIntParameter("y", 13);

    // Parses the buffer
    service_call_process(buffer, sizeof buffer, callbacks, LEN(callbacks));
}
