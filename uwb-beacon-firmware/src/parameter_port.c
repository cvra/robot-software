#include <ch.h>
#include <parameter/parameter_port.h>

static int buffer_allocated = 0;
// As only one buffer is allocated at a given time we can use a static
// buffer
static unsigned char buffer[256];

void parameter_port_lock(void)
{
    chSysLock();
}

void parameter_port_unlock(void)
{
    chSysUnlock();
}

void parameter_port_assert(int condition)
{
    chDbgAssert(condition, __FUNCTION__);
}

void* parameter_port_buffer_alloc(size_t size)
{
    parameter_port_lock();
    parameter_port_assert(buffer_allocated == 0);
    parameter_port_assert(size <= sizeof(buffer));
    buffer_allocated = 1;
    parameter_port_unlock();

    return buffer;
}

void parameter_port_buffer_free(void* p)
{
    parameter_port_lock();
    parameter_port_assert(buffer_allocated == 1);
    parameter_port_assert(p == buffer);
    buffer_allocated = 0;
    parameter_port_unlock();
}
