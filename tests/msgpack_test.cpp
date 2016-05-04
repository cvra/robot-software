#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "../parameter.h"
#include "../parameter_msgpack.h"
#include <cmp_mem_access/cmp_mem_access.h>
#include <cmp/cmp.h>
#include <string>

using namespace std;


static void msgpack_error_cb(void *arg, const char *id, const char *err)
{
    (void) arg;
    (void) id;
    FAIL(err);
}

TEST_GROUP(MessagePackTestGroup)
{
    parameter_namespace_t rootns;
    parameter_namespace_t a;
    parameter_t a_foo;
    parameter_t a_bar;
    parameter_t a_baz;

    char buffer[1024];
    cmp_mem_access_t mem;
    cmp_ctx_t ctx;

    void setup(void)
    {
        parameter_namespace_declare(&rootns, NULL, NULL);
        parameter_namespace_declare(&a, &rootns, "a");
        parameter_scalar_declare(&a_foo, &a, "foo");
        parameter_scalar_declare(&a_bar, &a, "bar");
        parameter_integer_declare(&a_baz, &a, "baz");

        cmp_mem_access_init(&ctx, &mem, buffer, sizeof buffer);
    }

    void teardown()
    {
        mock().clear();
    }
};

TEST(MessagePackTestGroup, CanChangeParameter)
{
    // Sets some default vaules
    parameter_scalar_set(&a_foo, 42);
    parameter_scalar_set(&a_bar, 60);

    // Clears the changed flag
    parameter_scalar_get(&a_foo);
    parameter_scalar_get(&a_bar);

    // Writes the update map: {'a': {'foo': 12.}}
    cmp_write_map(&ctx, 1);
    cmp_write_str(&ctx, "a", 1);
    cmp_write_map(&ctx, 1);
    cmp_write_str(&ctx, "foo", 3);
    cmp_write_float(&ctx, 12.);

    // Seek back to the beginning of the buffer
    cmp_mem_access_set_pos(&mem, 0);

    // Check that the namespace changed flag is cleared
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));

    // Update the parameter namespace
    parameter_msgpack_read_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    // Check that the namespace was changed
    CHECK_TRUE(parameter_namespace_contains_changed(&rootns));

    // /a/foo changed
    CHECK_EQUAL(12., parameter_scalar_get(&a_foo));

    // So now we don't have any changes left
    CHECK_FALSE(parameter_namespace_contains_changed(&rootns));

    // /a/bar wasn't changed
    CHECK_EQUAL(60., parameter_scalar_get(&a_bar));
}

TEST(MessagePackTestGroup, CanChangeMultipleParameters)
{
    cmp_write_map(&ctx, 1);
    cmp_write_str(&ctx, "a", 1);
    cmp_write_map(&ctx, 2);
    cmp_write_str(&ctx, "bar", 3);
    cmp_write_float(&ctx, 24.);
    cmp_write_str(&ctx, "foo", 3);
    cmp_write_float(&ctx, 12.);

    // Seek back to the beginning of the buffer
    cmp_mem_access_set_pos(&mem, 0);

    // Update the parameter namespace
    parameter_msgpack_read_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    CHECK_EQUAL(12., parameter_scalar_get(&a_foo));
    CHECK_EQUAL(24., parameter_scalar_get(&a_bar));
}

TEST(MessagePackTestGroup, TestWrite)
{
    uint32_t map_size;
    uint32_t name_len;
    int32_t ival;
    float val;
    char name[128];

    parameter_scalar_set(&a_foo, 42);
    parameter_scalar_set(&a_bar, 60);
    parameter_integer_set(&a_baz, 50);

    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    cmp_mem_access_set_pos(&mem, 0);

    // Root namespace has 1 value, which is a namespace
    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    CHECK_EQUAL(1, map_size);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("a", name);

    // "/a" has 3 values, all of them being parameters
    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    CHECK_EQUAL(3, map_size);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("baz", name);

    CHECK_TRUE(cmp_read_int(&ctx, &ival));
    CHECK_EQUAL(50, ival);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("bar", name);

    CHECK_TRUE(cmp_read_float(&ctx, &val));
    CHECK_EQUAL(60., val);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("foo", name);

    CHECK_TRUE(cmp_read_float(&ctx, &val));
    CHECK_EQUAL(42., val);
}

TEST(MessagePackTestGroup, TestUndefinedValuesAreIgnored)
{
    uint32_t size;
    char name[128];
    parameter_scalar_set(&a_bar, 40.);

    CHECK_FALSE(parameter_defined(&a_baz));
    CHECK_TRUE(parameter_defined(&a_bar));

    parameter_msgpack_write_cmp(&a, &ctx, msgpack_error_cb, NULL);
    cmp_mem_access_set_pos(&mem, 0);

    CHECK_TRUE(cmp_read_map(&ctx, &size));
    CHECK_EQUAL(1, size);

    size = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &size));
    STRCMP_EQUAL(name, "bar");
}

TEST(MessagePackTestGroup, TestWriteVector)
{
    uint32_t map_size;
    uint32_t name_len;

    char name[128];
    parameter_t array;
    float array_val[3];
    float array_init[] = {1., 2., 3.};

    parameter_namespace_declare(&rootns, NULL, NULL);
    parameter_namespace_declare(&a, &rootns, "a");
    parameter_vector_declare(&array, &a, "array", array_val, 3);
    parameter_vector_set(&array, array_init);

    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    cmp_mem_access_set_pos(&mem, 0);

    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));

    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("array", name);

    CHECK_TRUE(cmp_read_array(&ctx, &map_size));
    CHECK_EQUAL(3, map_size);

    // Check that the values are correctly taken
    for (int i = 0; i < 3; i++) {
        float val;
        CHECK_TRUE(cmp_read_float(&ctx, &val));
        CHECK_EQUAL(array_init[i], val);
    }
}

TEST(MessagePackTestGroup, TestWriteVariableVector)
{
    uint32_t map_size;
    uint32_t name_len;

    char name[128];
    parameter_t array;
    float array_val[5];
    float array_init[] = {1., 2., 3.};

    parameter_namespace_declare(&rootns, NULL, NULL);
    parameter_namespace_declare(&a, &rootns, "a");
    parameter_variable_vector_declare(&array, &a, "array", array_val, 5);

    parameter_variable_vector_set(&array, array_init, 3);

    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    cmp_mem_access_set_pos(&mem, 0);

    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));

    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("array", name);

    CHECK_TRUE(cmp_read_array(&ctx, &map_size));
    CHECK_EQUAL(3, map_size);

    // Check that the values are correctly taken
    for (int i = 0; i < 3; i++) {
        float val;
        CHECK_TRUE(cmp_read_float(&ctx, &val));
        CHECK_EQUAL(array_init[i], val);
    }
}

TEST(MessagePackTestGroup, TestWriteString)
{
    uint32_t map_size;
    uint32_t name_len;

    parameter_t array;
    char array_value[128];

    char name[128];

    parameter_namespace_declare(&rootns, NULL, NULL);

    parameter_string_declare(&array, &rootns, "str",
                              array_value, sizeof(array_value));

    parameter_string_set(&array, "hello");

    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    cmp_mem_access_set_pos(&mem, 0);

    CHECK_TRUE(cmp_read_map(&ctx, &map_size));
    CHECK_EQUAL(1, map_size);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("str", name);

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));
    STRCMP_EQUAL("hello", name);
}

TEST(MessagePackTestGroup, TestBooleanWrite)
{
    char name[128];

    parameter_namespace_declare(&rootns, NULL, NULL);
    parameter_boolean_declare(&a_foo, &rootns, "foo");
    parameter_boolean_set(&a_foo, true);

    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    cmp_mem_access_set_pos(&mem, 0);

    uint32_t map_size, name_len;
    CHECK_TRUE(cmp_read_map(&ctx, &map_size));

    name_len = sizeof(name);
    CHECK_TRUE(cmp_read_str(&ctx, name, &name_len));

    bool res;
    CHECK_TRUE(cmp_read_bool(&ctx, &res));
    CHECK_EQUAL(true, res);
}

TEST(MessagePackTestGroup, TestBooleanRead)
{
    parameter_namespace_declare(&rootns, NULL, NULL);
    parameter_boolean_declare(&a_foo, &rootns, "foo");

    cmp_write_map(&ctx, 1);
    cmp_write_str(&ctx, "foo", 3);
    cmp_write_bool(&ctx, true);

    cmp_mem_access_set_pos(&mem, 0);

    parameter_msgpack_read_cmp(&rootns, &ctx, msgpack_error_cb, NULL);
    CHECK_TRUE(parameter_defined(&a_foo));
    CHECK_EQUAL(true, parameter_boolean_get(&a_foo));
}

void log_problematic_id_callback(void *p, const char *id, const char *err)
{
    (void) p;
    (void) err;

    mock().actualCall("error").withParameter("id", id);
}

TEST(MessagePackTestGroup, TestWriteUnknownParameter)
{
    parameter_t bad_param;

    parameter_namespace_declare(&rootns, NULL, NULL);

    parameter_integer_declare_with_default(&bad_param, &rootns, "bad_param", 40);

    // Bogus type
    bad_param.type = 99;

    mock().expectOneCall("error").withParameter("id", "bad_param");
    parameter_msgpack_write_cmp(&rootns, &ctx, log_problematic_id_callback, NULL);

    mock().checkExpectations();
}

TEST(MessagePackTestGroup, TestWriteNotEnoughSpaceLeft)
{
    parameter_t param;

    // Have a fake write buffer with only 2 bytes left
    cmp_mem_access_init(&ctx, &mem, buffer, 2);

    parameter_namespace_declare(&rootns, NULL, NULL);

    parameter_integer_declare(&param, &rootns, "param");
    parameter_integer_set(&param, 14);

    mock().expectOneCall("error").withParameter("id", "param");
    parameter_msgpack_write_cmp(&rootns, &ctx, log_problematic_id_callback, NULL);

    mock().checkExpectations();
}

TEST(MessagePackTestGroup, TestBackAndForth)
{
    // Write an initial value
    parameter_integer_set(&a_baz, 42);

    // Save it as messagepack
    parameter_msgpack_write_cmp(&rootns, &ctx, msgpack_error_cb, NULL);
    cmp_mem_access_set_pos(&mem, 0);

    // Change it
    parameter_integer_set(&a_baz, 99);

    // Load it from messagepack
    parameter_msgpack_read_cmp(&rootns, &ctx, msgpack_error_cb, NULL);

    // Check that it had its old value
    CHECK_EQUAL(42, parameter_integer_get(&a_baz));
}

TEST(MessagePackTestGroup, IgnoresUnknownParameters)
{
    parameter_scalar_set(&a_foo, 42);

    cmp_write_map(&ctx, 1);
    cmp_write_str(&ctx, "a", 1);
    cmp_write_map(&ctx, 2);
    cmp_write_str(&ctx, "b", 1);
    cmp_write_float(&ctx, 38.);
    cmp_write_str(&ctx, "foo", 3);
    cmp_write_float(&ctx, 12.);

    cmp_mem_access_set_pos(&mem, 0);

    parameter_msgpack_read_cmp(&rootns, &ctx, NULL, NULL);

    CHECK_EQUAL(12., parameter_scalar_get(&a_foo));
}
