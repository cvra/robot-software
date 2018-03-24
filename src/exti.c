#include <ch.h>
#include <hal.h>
#include <trace/trace.h>
#include "trace_points.h"

#include "exti.h"

EVENTSOURCE_DECL(exti_imu_event);
EVENTSOURCE_DECL(exti_uwb_event);

static void exti_cb(EXTDriver *extp, expchannel_t channel)
{
    (void) extp;

    if (channel == GPIOB_IMU_INT) {
        chSysLockFromISR();
        chEvtBroadcastI(&exti_imu_event);
        chSysUnlockFromISR();
    } else if (channel == GPIOA_UWB_INT) {
        trace(TRACE_POINT_UWB_IRQ);
        chSysLockFromISR();
        chEvtBroadcastI(&exti_uwb_event);
        chSysUnlockFromISR();
    }
}


static const EXTConfig extcfg = {
    {
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},

        /* DWM 1000 IRQ. */
        {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, exti_cb},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},

        /* MPU9250 IRQ */
        {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, exti_cb},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

void exti_start(void)
{
    extStart(&EXTD1, &extcfg);
}
