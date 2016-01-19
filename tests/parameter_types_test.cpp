#include "CppUTest/TestHarness.h"
#include "../parameter.h"

TEST_GROUP(IntegerParamter)
{
    parameter_namespace_t ns;
    parameter_t p;

    void setup(void)
    {
        parameter_namespace_declare(&ns, NULL, NULL);
        parameter_integer_declare(&p, &ns, "int");
    }
};

TEST(IntegerParamter, TypeFlag)
{
    CHECK_EQUAL(_PARAM_TYPE_INTEGER, p.type);
}

TEST(IntegerParamter, CanSet)
{
    parameter_integer_set(&p, 42);
    CHECK_TRUE(parameter_changed(&p));
    CHECK_EQUAL(42, p.value.i);
}

TEST(IntegerParamter, CanGet)
{
    parameter_integer_set(&p, 42);
    CHECK_EQUAL(42, parameter_integer_get(&p));
    CHECK_FALSE(parameter_changed(&p));
}

TEST(IntegerParamter, CanRead)
{
    parameter_integer_set(&p, 42);
    CHECK_EQUAL(42, parameter_integer_read(&p));
    CHECK_TRUE(parameter_changed(&p));
}

TEST(IntegerParamter, CanDeclareWithDefault)
{
    parameter_t p_default;
    parameter_integer_declare_with_default(&p_default, &ns, "defaultint", 1337);
    CHECK_TRUE(parameter_changed(&p_default));
    CHECK_EQUAL(1337, parameter_integer_get(&p_default));
}



TEST_GROUP(StringParamter)
{
    parameter_namespace_t ns;
    parameter_t p;
    char str_buffer[8];

    void setup(void)
    {
        parameter_namespace_declare(&ns, NULL, NULL);
        parameter_string_declare(&p, &ns, "str", str_buffer, sizeof(str_buffer));
    }
};

TEST(StringParamter, TypeFlag)
{
    CHECK_EQUAL(_PARAM_TYPE_STRING, p.type);
}

TEST(StringParamter, BufLen)
{
    CHECK_EQUAL(sizeof(str_buffer), p.value.str.buf_len);
}

TEST(StringParamter, CanSet)
{
    parameter_string_set(&p, "hello");
    CHECK_TRUE(parameter_changed(&p));
    BYTES_EQUAL('h', p.value.str.buf[0]);
    BYTES_EQUAL('e', p.value.str.buf[1]);
    BYTES_EQUAL('l', p.value.str.buf[2]);
    BYTES_EQUAL('l', p.value.str.buf[3]);
    BYTES_EQUAL('o', p.value.str.buf[4]);
    CHECK_EQUAL(5, p.value.str.len);
}

TEST(StringParamter, CanGet)
{
    parameter_string_set(&p, "hello");
    char buf[9];
    parameter_string_get(&p, buf, sizeof(buf));
    STRCMP_EQUAL("hello", buf);
    CHECK_FALSE(parameter_changed(&p));
}

TEST(StringParamter, CanRead)
{
    parameter_string_set(&p, "hello");
    char buf[9];
    int len = parameter_string_read(&p, buf, sizeof(buf));
    CHECK_EQUAL(5, len);
    STRCMP_EQUAL("hello", buf);
    CHECK_TRUE(parameter_changed(&p));
}

TEST(StringParamter, CanGetSmallBuf)
{
    parameter_string_set(&p, "hello");
    char buf[3]; // "he"+'\0'
    int len = parameter_string_get(&p, buf, sizeof(buf));
    CHECK_EQUAL(5, len);
    STRCMP_EQUAL("he", buf);
    CHECK_FALSE(parameter_changed(&p));
}

TEST(StringParamter, CanDeclareWithDefault)
{
    parameter_t p_default;
    parameter_string_declare_with_default(&p_default, &ns, "defaultstr",
            str_buffer, sizeof(str_buffer), "#default");
    char buf[9];
    parameter_string_read(&p_default, buf, sizeof(buf));
    STRCMP_EQUAL("#default", buf);
    CHECK_TRUE(parameter_changed(&p_default));
}

TEST_GROUP(BoolParameter)
{
    parameter_namespace_t ns;
    parameter_t p;

    void setup(void)
    {
        parameter_namespace_declare(&ns, NULL, NULL);
        parameter_boolean_declare(&p, &ns, "bool");
    }
};

TEST(BoolParameter, TypeFlag)
{
    CHECK_EQUAL(_PARAM_TYPE_BOOLEAN, p.type);
    STRCMP_EQUAL("bool", p.id);
}

TEST(BoolParameter, CanSet)
{
    parameter_boolean_set(&p, true);
    CHECK_TRUE(parameter_changed(&p));
    CHECK_EQUAL(true, p.value.b);
}

TEST(BoolParameter, CanRead)
{
    parameter_boolean_set(&p, true);
    CHECK_EQUAL(true, parameter_boolean_read(&p));
    CHECK_TRUE(parameter_changed(&p));
}

TEST(BoolParameter, CanGet)
{
    parameter_boolean_set(&p, true);
    CHECK_EQUAL(true, parameter_boolean_get(&p));
    CHECK_FALSE(parameter_changed(&p));
}

TEST(BoolParameter, CanDeclareWithDefault)
{
    parameter_t p_default;
    parameter_boolean_declare_with_default(&p_default, &ns, "defaultbool", true);
    CHECK_TRUE(parameter_changed(&p_default));
    CHECK_EQUAL(true, parameter_boolean_get(&p_default));
}
