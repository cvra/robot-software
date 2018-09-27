#include <ch.h>
#include <osal.h>
#include <parameter/parameter_port.h>

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
    osalDbgAssert(condition, "parameter_assert");
}

/* parameter guarantees that only 1 buffer will be allocated at a single time.
 * It also says that the size is at most the length of the longer
 * param/namespace name or the size of the biggest array matrix or string. */
static char param_buffer[64];
void *parameter_port_buffer_alloc(size_t size)
{
    osalDbgAssert(size <= sizeof(param_buffer), "Parameter buffer too big");
    return param_buffer;
}

void parameter_port_buffer_free(void *buffer)
{
    (void) buffer;
}
