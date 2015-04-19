#include "CppUTest/TestHarness.h"
#include "../parameter.h"
#include "../parameter_msgpack.h"
#include <cmp_mem_access/cmp_mem_access.h>
#include <cmp/cmp.h>
#include <string>

using namespace std;


static void msgpack_error_cb(void *arg, const char *id, const char *err)
{
    FAIL(err);
}

TEST_GROUP(MessagePackTestGroup)
{
    parameter_namespace_t rootns;
    parameter_namespace_t a;
    parameter_t a_foo;
    parameter_t a_bar;

    char buffer[1024];
    cmp_mem_access_t mem;
    cmp_ctx_t ctx;

    void setup(void)
    {
        parameter_namespace_declare(&rootns, NULL, NULL);
        parameter_namespace_declare(&a, &rootns, "a");
        parameter_scalar_declare(&a_foo, &a, "foo");
        parameter_scalar_declare(&a_bar, &a, "bar");

        cmp_mem_access_init(&ctx, &mem, buffer, sizeof buffer);
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
