#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../service_call.h"
#include <cstring>

#define LEN(a) (sizeof (a) / sizeof(a[0]))

void foo_cb(void *p, int argc, cmp_ctx_t *args_ctx, cmp_ctx_t *output_ctx)
{
    mock().actualCall("foo").withIntParameter("argc", argc);
}

void bar_cb(void *p, int argc, cmp_ctx_t *args_ctx, cmp_ctx_t *output_ctx)
{
    mock().actualCall("bar");
}

/** This callback shows how to read the several arguments. */
void arg_read_cb(void *p, int argc, cmp_ctx_t *args_ctx, cmp_ctx_t *output_ctx)
{
    bool result;
    int arg;
    char arg_name[64];
    unsigned int arg_len;

    MockActualCall& state = mock().actualCall("arg_read_cb");

    result = cmp_read_int(args_ctx, &arg);
    CHECK_TRUE(result);
    state = state.withIntParameter("x", arg);

    result = cmp_read_int(args_ctx, &arg);
    CHECK_TRUE(result);
    state = state.withIntParameter("y", arg);
}

void output_test_cb(void *p, int argc, cmp_ctx_t *args_ctx, cmp_ctx_t *output_ctx)
{
    cmp_write_str(output_ctx, "hello", 5);
}


service_call_method callbacks[] = {
    {.name = "foo", .cb = foo_cb},
    {.name = "bar", .cb = bar_cb},
    {.name = "arg_read", .cb = arg_read_cb},
    {.name = "output_test_cb", .cb = output_test_cb}
};


TEST_GROUP(ServiceCallTestGroup)
{
    uint8_t buffer[1024];
    uint8_t output_buffer[64];
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
    uint32_t array_size;

    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 12);

    cmp_mem_access_set_pos(&mem, 0);

    result = cmp_read_array(&ctx, &array_size);
    CHECK_TRUE(result);
    CHECK_EQUAL(12 + 1, array_size);

    result = cmp_read_str(&ctx, method_name, &size);
    CHECK_TRUE(result);
    STRCMP_EQUAL(method_name, "foo");
}

TEST(ServiceCallTestGroup, CanProcessServiceCall)
{
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 0);

    mock().expectOneCall("foo").withIntParameter("argc", 0);
    service_call_process(buffer, sizeof buffer, NULL, 0, callbacks, LEN(callbacks));
}

TEST(ServiceCallTestGroup, CallsCorrectServiceCall)
{
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "bar", 0);

    mock().expectOneCall("bar");

    service_call_process(buffer, sizeof buffer, NULL, 0, callbacks, LEN(callbacks));
}

TEST(ServiceCallTestGroup, PassesArgcCorrectly)
{
    const int argc = 2;

    // Writes the header with two arguments
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "arg_read", argc);

    // Writes the arguments
    cmp_write_uint(&ctx, 12);
    cmp_write_uint(&ctx, 13);

    mock().expectOneCall("arg_read_cb")
          .withIntParameter("x", 12)
          .withIntParameter("y", 13);

    // Parses the buffer
    service_call_process(buffer, sizeof buffer, NULL, 0, callbacks, LEN(callbacks));
}

static void pointer_arg_cb(void *p, int argc, cmp_ctx_t *args_ctx, cmp_ctx_t *output_ctx)
{
    mock().actualCall("cb").withPointerParameter("p", p);
}

TEST(ServiceCallTestGroup, PassesPointerCorrectly)
{
    int foo;
    service_call_method callbacks[] = {
        {.name = "cb", .cb = pointer_arg_cb, .arg=&foo},
    };

    mock().expectOneCall("cb").withPointerParameter("p", &foo);

    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "cb", 0);

    // Parses the buffer
    service_call_process(buffer, sizeof buffer, NULL, 0, callbacks, LEN(callbacks));
}


TEST(ServiceCallTestGroup, OutputTest)
{
    const int argc = 0;

    char str[10];
    unsigned int str_len = sizeof str;
    bool result;

    // Writes header
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "output_test_cb", argc);

    service_call_process(buffer, sizeof buffer,
                         output_buffer, sizeof output_buffer,
                         callbacks, LEN(callbacks));

    cmp_mem_access_ro_init(&ctx, &mem, output_buffer, sizeof output_buffer);
    result = cmp_read_str(&ctx, str, &str_len);

    CHECK_TRUE(result);
    STRCMP_EQUAL(str, "hello");
}

TEST(ServiceCallTestGroup, OutputLenIsReturned)
{
    size_t output_len;
    service_call_encode(&ctx, &mem, buffer, sizeof buffer, "output_test_cb", 0);

    output_len = service_call_process(buffer, sizeof buffer,
                                      output_buffer, sizeof output_buffer,
                                      callbacks, LEN(callbacks));

    CHECK_EQUAL(6, output_len);
}
