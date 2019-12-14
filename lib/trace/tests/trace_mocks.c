#include <stdint.h>

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
