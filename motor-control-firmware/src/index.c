#include <hal.h>
#include "index.h"
#include "control.h"

static float position;
static uint32_t update_count;

static void index_cb(void* arg)
{
    (void)arg;

    chSysLockFromISR();

    position = control_get_position();
    update_count++;

    chSysUnlockFromISR();
}

void index_init(void)
{
    position = 0;

    // Enable event on index pin, active low, PA12
    palEnableLineEvent(PAL_LINE(GPIOA, 12U), PAL_EVENT_MODE_FALLING_EDGE);
    palSetLineCallback(PAL_LINE(GPIOA, 12U), index_cb, NULL);
}

void index_get_position(float* out_position, uint32_t* out_update_count)
{
    *out_position = position;
    *out_update_count = update_count;
}
