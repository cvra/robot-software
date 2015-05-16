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


