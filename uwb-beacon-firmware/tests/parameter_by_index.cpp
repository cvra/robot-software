#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../src/uavcan/parameter_enumeration.hpp"

TEST_GROUP (ParameterByIndexTestGroup) {
    parameter_namespace_t ns;
    parameter_t foo, bar, baz;
    parameter_namespace_t sub_ns;
    parameter_t sub_param_1, sub_param_2;
    void setup(void)
    {
        parameter_namespace_declare(&ns, NULL, "");

        parameter_boolean_declare(&bar, &ns, "bar");
        parameter_boolean_declare(&baz, &ns, "baz");
        parameter_boolean_declare(&foo, &ns, "foo");

        parameter_namespace_declare(&sub_ns, &ns, "sub");
        parameter_boolean_declare(&sub_param_1, &sub_ns, "sub1");
        parameter_boolean_declare(&sub_param_2, &sub_ns, "sub1");
    }
};

TEST(ParameterByIndexTestGroup, TrivialCase)
{
    auto p = parameter_find_by_index(&ns, 0);
    POINTERS_EQUAL(&foo, p);
}

TEST(ParameterByIndexTestGroup, NonRecursiveCase)
{
    auto p = parameter_find_by_index(&ns, 2);
    POINTERS_EQUAL(&bar, p);
}

TEST(ParameterByIndexTestGroup, AfterLastParameter)
{
    auto p = parameter_find_by_index(&ns, 100);
    POINTERS_EQUAL(nullptr, p);
}

TEST(ParameterByIndexTestGroup, RecursiveCase)
{
    POINTERS_EQUAL(&sub_param_2, parameter_find_by_index(&ns, 3));
    POINTERS_EQUAL(&sub_param_1, parameter_find_by_index(&ns, 4));
}

TEST(ParameterByIndexTestGroup, TestHeight)
{
    CHECK_EQUAL(1, parameter_tree_height(&foo));
    CHECK_EQUAL(2, parameter_tree_height(&sub_param_1));
}
