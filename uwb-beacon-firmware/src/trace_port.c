#include <ch.h>
#include <hal.h>
#include <trace/trace.h>
#include "trace_points.h"

extern int32_t trace_lock(void)
{
    return chSysGetStatusAndLockX();
}

extern void trace_unlock(int32_t status)
{
    chSysRestoreStatusX(status);
}

extern int32_t trace_timestamp_ms_get(void)
{
    return TIME_I2MS(chVTGetSystemTimeX());
}

#undef C
#define C(x) #x,

const char* trace_point_names[] = {
    TRACE_POINTS};
