#include "scara/scara_port.h"

static int32_t time_us;

void scara_time_set(int32_t time)
{
    time_us = time;
}

int32_t scara_time_get_impl(void)
{
    return time_us;
}

void scara_panic_impl(void)
{

}

int32_t (*scara_time_get)(void) = scara_time_get_impl;
void (*scara_panic)(void) = scara_panic_impl;
