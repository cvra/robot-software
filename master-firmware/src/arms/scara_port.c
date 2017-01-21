#include <ch.h>
#include "timestamp/timestamp.h"
#include "scara/scara_port.h"

int32_t scara_time_get_impl(void)
{
    return timestamp_get();
}

void scara_panic_impl(void)
{
    chSysHalt("Arm panic");
}

int32_t (*scara_time_get)(void) = scara_time_get_impl;
void (*scara_panic)(void) = scara_panic_impl;
