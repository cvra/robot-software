#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "parameter/parameter.h"
#include "parameter/parameter_port.h"
#include <stdlib.h>

void parameter_port_lock(void)
{
}

void parameter_port_unlock(void)
{
}

void parameter_port_assert(int condition)
{
    CHECK_TRUE(condition);
}

void *parameter_port_buffer_alloc(size_t size)
{
    return malloc(size);
}

void parameter_port_buffer_free(void *buffer)
{
    free(buffer);
}
