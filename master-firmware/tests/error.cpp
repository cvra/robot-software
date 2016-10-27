#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>

extern "C" {
#include "error/error.h"
}

TEST_GROUP(ErrorLoggingTestGroup)
{
    void teardown(void)
    {
        error_register_error(NULL);
        error_register_warning(NULL);
        error_register_notice(NULL);
        error_register_debug(NULL);
    }
};

void error_mock(struct error *e, ...)
{
    mock().actualCall("error")
          .withIntParameter("err_num", e->err_num)
          .withIntParameter("severity", e->severity)
          .withStringParameter("file", e->file)
          .withIntParameter("line", e->line)
          .withStringParameter("text", e->text);
}

TEST(ErrorLoggingTestGroup, PopulatesFieldError)
{
    auto error = error_generate(42, ERROR_SEVERITY_WARNING,
                                "foo", __FILE__, 120);

    CHECK_EQUAL(42, error.err_num);
    CHECK_EQUAL(ERROR_SEVERITY_WARNING, error.severity);
    STRCMP_EQUAL("foo", error.text);
    CHECK_EQUAL(120, error.line);
    STRCMP_EQUAL(__FILE__, error.file);
}

TEST(ErrorLoggingTestGroup, MockWorks)
{
    auto error = error_generate(42, ERROR_SEVERITY_WARNING,
                                "foo", __FILE__, 120);
    mock().expectOneCall("error")
          .withIntParameter("err_num", 42)
          .withIntParameter("severity", ERROR_SEVERITY_WARNING)
          .withIntParameter("line", 120)
          .withStringParameter("text", "foo")
          .withStringParameter("file", __FILE__);
    error_mock(&error);
}

TEST(ErrorLoggingTestGroup, GeneratesError)
{
    error_register_error(error_mock);
    mock().expectOneCall("error")
          .withIntParameter("err_num", 42)
          .withIntParameter("severity", ERROR_SEVERITY_ERROR)
          .withIntParameter("line", __LINE__ + 4)
          .withStringParameter("text", "foo")
          .withStringParameter("file", __FILE__);

    ERROR(42, "foo");
}

TEST(ErrorLoggingTestGroup, GeneratesWarning)
{
    error_register_warning(error_mock);
    mock().expectOneCall("error")
          .withIntParameter("err_num", 42)
          .withIntParameter("severity", ERROR_SEVERITY_WARNING)
          .withIntParameter("line", __LINE__ + 4)
          .withStringParameter("text", "foo")
          .withStringParameter("file", __FILE__);

    WARNING(42, "foo");
}

TEST(ErrorLoggingTestGroup, GeneratesNotice)
{
    error_register_notice(error_mock);
    mock().expectOneCall("error")
          .withIntParameter("err_num", 42)
          .withIntParameter("severity", ERROR_SEVERITY_NOTICE)
          .withIntParameter("line", __LINE__ + 4)
          .withStringParameter("text", "foo")
          .withStringParameter("file", __FILE__);

    NOTICE(42, "foo");
}

TEST(ErrorLoggingTestGroup, GeneratesDebug)
{
    error_register_debug(error_mock);
    mock().expectOneCall("error")
          .withIntParameter("err_num", 42)
          .withIntParameter("severity", ERROR_SEVERITY_DEBUG)
          .withIntParameter("line", __LINE__ + 4)
          .withStringParameter("text", "foo")
          .withStringParameter("file", __FILE__);

    DEBUG(42, "foo");
}
