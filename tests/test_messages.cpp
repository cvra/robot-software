#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../message.h"


TEST_GROUP(MessageTestGroup)
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

TEST(MessageTestGroup, CanEncodeMessage)
{
    char method_name[4];
    bool result;
    uint32_t size = sizeof method_name;
    uint32_t array_size;

    message_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 2);
    cmp_mem_access_set_pos(&mem, 0);

    result = cmp_read_array(&ctx, &array_size);
    CHECK_TRUE(result);
    CHECK_EQUAL(3, array_size);

    result = cmp_read_str(&ctx, method_name, &size);
    CHECK_TRUE(result);
    STRCMP_EQUAL("foo", method_name);
}

static void argc_cb(void *p, int argc, cmp_ctx_t *ctx)
{
    mock().actualCall("argc").withIntParameter("argc", argc);
}

static void bar_cb(void *p, int argc, cmp_ctx_t *ctx)
{
    int x, y;
    bool result;

    result = cmp_read_int(ctx, &x);
    CHECK_TRUE(result);
    result = cmp_read_int(ctx, &y);
    CHECK_TRUE(result);

    mock().actualCall("bar").withIntParameter("x", x).withIntParameter("y", y);
}

static message_method_t callbacks[] = {
    {.name="argc", .cb=argc_cb, .arg=NULL},
    {.name="bar", .cb=bar_cb, .arg=NULL},
};

TEST(MessageTestGroup, CanCallCallback)
{
    mock().expectOneCall("argc").withIntParameter("argc", 10);

    message_encode(&ctx, &mem, buffer, sizeof buffer, "argc", 10);
    message_process(buffer, sizeof buffer, callbacks, sizeof(callbacks) / sizeof(callbacks[0]));
}

TEST(MessageTestGroup, CanUseArguments)
{
    mock().expectOneCall("bar").withIntParameter("x", 2).withIntParameter("y", 4);
    message_encode(&ctx, &mem, buffer, sizeof buffer, "bar", 2);

    // Write arguments
    cmp_write_sint(&ctx, 2);
    cmp_write_sint(&ctx, 4);

    message_process(buffer, sizeof buffer, callbacks, sizeof(callbacks) / sizeof(callbacks[0]));
}

static void first_cb(void *p, int argc, cmp_ctx_t *ctx)
{
    (void) argc;
    (void) ctx;
    mock().actualCall("first");
}

static void second_cb(void *p, int argc, cmp_ctx_t *ctx)
{
    (void) argc;
    (void) ctx;
    mock().actualCall("second");
}

TEST(MessageTestGroup, ExitAfterCallbackFound)
{
    /* This test checks that only the first callback is called. */
    static message_method_t callbacks[] = {
        {.name="foo", .cb=first_cb, .arg=NULL},
        {.name="foo", .cb=second_cb, .arg=NULL},
    };

    mock().expectOneCall("first");
    // No call to second callback is expected

    message_encode(&ctx, &mem, buffer, sizeof buffer, "foo", 0);
    message_process(buffer, sizeof buffer, callbacks, 2);
}

static void arg_cb(void *arg, int argc, cmp_ctx_t *ctx)
{
    (void) argc;
    (void) ctx;
    mock().actualCall("cb").withPointerParameter("arg", arg);
}

TEST(MessageTestGroup, ArgIsForwarded)
{
    int foo;
    message_method_t callbacks[] = {
        {.name="cb", .cb=arg_cb, .arg=&foo}
    };
    mock().expectOneCall("cb").withPointerParameter("arg", &foo);

    message_encode(&ctx, &mem, buffer, sizeof buffer, "cb", 0);
    message_process(buffer, sizeof buffer, callbacks, 1);
}
