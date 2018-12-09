#include <ch.h>
#include <hal.h>
#include <trace/trace.h>
#include "trace_points.h"

EVENTSOURCE_DECL(imu_event);
EVENTSOURCE_DECL(uwb_event);

static void imu_cb(void* arg)
{
    (void)arg;

    chSysLockFromISR();

    chEvtBroadcastI(&imu_event);

    chSysUnlockFromISR();
}

static void uwb_cb(void* arg)
{
    (void)arg;
    trace(TRACE_POINT_UWB_IRQ);

    chSysLockFromISR();

    chEvtBroadcastI(&uwb_event);

    chSysUnlockFromISR();
}

void exti_start(void)
{
    // Enable event on DWM 1000 IRQ, PA8
    palEnableLineEvent(PAL_LINE(GPIOA, 8U), PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(PAL_LINE(GPIOA, 8U), uwb_cb, NULL);

    // Enable event on MPU9250, PB11
    palEnableLineEvent(PAL_LINE(GPIOB, 11U), PAL_EVENT_MODE_RISING_EDGE);
    palSetLineCallback(PAL_LINE(GPIOB, 11U), imu_cb, NULL);
}
