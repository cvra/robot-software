#include <string.h>
#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "trace/trace.h"
#include "trace_points.h"

extern volatile struct trace_buffer_struct trace_buffer;

int32_t trace_timestamp_ms_get(void)
{
    return 1234;
}

int32_t trace_lock(void)
{
    return 0;
}

void trace_unlock(int32_t status)
{
}

TEST_GROUP (TraceTestGroup) {
    void setup()
    {
        trace_init();
        trace_enable();
    }
};

TEST(TraceTestGroup, CorrectInit)
{
    CHECK_EQUAL(0, trace_buffer.nb_events);
    CHECK_EQUAL(0, trace_buffer.write_index);
    CHECK_EQUAL(true, trace_buffer.active);
}

TEST(TraceTestGroup, CanTraceSimpleEvent)
{
    trace(TRACE_POINT_1);
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_STRING, trace_buffer.data[0].type);
    CHECK_EQUAL(1234, trace_buffer.data[0].timestamp);
    STRCMP_EQUAL("", trace_buffer.data[0].data.string);
    CHECK_EQUAL(1, trace_buffer.nb_events);
    CHECK_EQUAL(1, trace_buffer.write_index);
}

TEST(TraceTestGroup, CanTraceEventWithMessage)
{
    trace_string(TRACE_POINT_1, "trace message");
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_STRING, trace_buffer.data[0].type);
    CHECK_EQUAL(1234, trace_buffer.data[0].timestamp);
    STRCMP_EQUAL("trace message", trace_buffer.data[0].data.string);
}

TEST(TraceTestGroup, CanTraceEventWithInteger)
{
    trace_integer(TRACE_POINT_1, 0xC0FFEE);
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_INTEGER, trace_buffer.data[0].type);
    CHECK_EQUAL(1234, trace_buffer.data[0].timestamp);
    CHECK_EQUAL(0xC0FFEE, trace_buffer.data[0].data.integer);
}

TEST(TraceTestGroup, CanTraceEventWithScalar)
{
    trace_scalar(TRACE_POINT_1, 3.141);
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_SCALAR, trace_buffer.data[0].type);
    CHECK_EQUAL(1234, trace_buffer.data[0].timestamp);
    CHECK_EQUAL((float)3.141, trace_buffer.data[0].data.scalar);
}

TEST(TraceTestGroup, CanTraceEventWithAddress)
{
    int data;
    trace_address(TRACE_POINT_1, &data);
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_ADDRESS, trace_buffer.data[0].type);
    CHECK_EQUAL(1234, trace_buffer.data[0].timestamp);
    CHECK_EQUAL(&data, trace_buffer.data[0].data.address);
}

TEST(TraceTestGroup, CanTraceMultipleEvents)
{
    trace_integer(TRACE_POINT_0, 101);
    CHECK_EQUAL(TRACE_POINT_0, trace_buffer.data[0].event_id);

    trace_integer(TRACE_POINT_1, 202);
    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[1].event_id);

    trace_integer(TRACE_POINT_2, 303);
    CHECK_EQUAL(TRACE_POINT_2, trace_buffer.data[2].event_id);

    CHECK_EQUAL(3, trace_buffer.nb_events);
    CHECK_EQUAL(3, trace_buffer.write_index);
}

TEST(TraceTestGroup, CorrectRingbufferOverwrite)
{
    // fill trace buffer
    int i;
    for (i = 0; i < TRACE_BUFFER_SIZE; i++) {
        trace_integer(TRACE_POINT_0, 0xFFFFFFFF);
    }

    CHECK_EQUAL(TRACE_BUFFER_SIZE, trace_buffer.nb_events);
    CHECK_EQUAL(0, trace_buffer.write_index);

    // wrap around & overwrite first element
    trace_string(TRACE_POINT_1, "hello world");

    CHECK_EQUAL(TRACE_POINT_0, trace_buffer.data[TRACE_BUFFER_SIZE - 1].event_id);

    CHECK_EQUAL(TRACE_POINT_1, trace_buffer.data[0].event_id);
    CHECK_EQUAL(TRACE_TYPE_STRING, trace_buffer.data[0].type);
    STRCMP_EQUAL("hello world", trace_buffer.data[0].data.string);

    CHECK_EQUAL(TRACE_BUFFER_SIZE, trace_buffer.nb_events);
    CHECK_EQUAL(1, trace_buffer.write_index);
}

extern "C" void print_fn(void* p, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char** buf = (char**)p;
    int n = vsprintf(*buf, fmt, ap);
    if (n > 0) {
        *buf += n;
    }
    va_end(ap);
}

TEST_GROUP (TracePrintTestGroup) {
    char buffer[1000];
    void* arg;
    void setup()
    {
        memset(buffer, 0, sizeof(buffer));
        arg = buffer;
        trace_init();
        trace_enable();
    }
};

TEST(TracePrintTestGroup, CanPrintEventWithInteger)
{
    trace_integer(TRACE_POINT_1, 42);
    trace_print(print_fn, &arg);
    STRCMP_EQUAL("[1234] TRACE_POINT_1: 42\n", buffer);
}

TEST(TracePrintTestGroup, CanPrintEventWithMessage)
{
    trace_string(TRACE_POINT_0, "hello world");
    trace_print(print_fn, &arg);
    STRCMP_EQUAL("[1234] TRACE_POINT_0: \"hello world\"\n", buffer);
}

TEST(TracePrintTestGroup, CanPrintEventWithScalar)
{
    trace_scalar(TRACE_POINT_2, 1.5);
    trace_print(print_fn, &arg);
    STRCMP_EQUAL("[1234] TRACE_POINT_2: 1.500000\n", buffer);
}

TEST(TracePrintTestGroup, CanPrintEventWithAddr)
{
    int foo;
    trace_address(TRACE_POINT_0, &foo);
    trace_print(print_fn, &arg);
    char buf[100];
    snprintf(buf, sizeof(buf), "[1234] TRACE_POINT_0: %p\n", &foo);
    STRCMP_EQUAL(buf, buffer);
}

TEST(TracePrintTestGroup, CanPrintMultipleEventWithInteger)
{
    trace_string(TRACE_POINT_0, "hello");
    trace_integer(TRACE_POINT_1, 42);
    trace_scalar(TRACE_POINT_2, 1);
    trace_print(print_fn, &arg);
    STRCMP_EQUAL(
        "[1234] TRACE_POINT_0: \"hello\"\n"
        "[1234] TRACE_POINT_1: 42\n"
        "[1234] TRACE_POINT_2: 1.000000\n",
        buffer);
}
