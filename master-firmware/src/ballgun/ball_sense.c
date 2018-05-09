#include <ch.h>
#include <hal.h>

#include <error/error.h>

#include "ball_sensor.h"
#include "ball_sense.h"

#define BALL_SENSE_STACKSIZE 512

static ball_sensor_t ball_sensor;

static inline bool ball_sense_read(void)
{
    return palReadPad(GPIOG, GPIOG_PIN9);
}

static THD_FUNCTION(ball_sense_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    ball_sensor_init(&ball_sensor);
    ball_sensor_configure(&ball_sensor, true, 2, 3);

    NOTICE("Ball sensor ready to count balls");

    while (true) {
        bool measurement = ball_sense_read();
        ball_sensor_manage(&ball_sensor, measurement);
        chThdSleepMilliseconds(1);
    }
}

void ball_sense_start(void)
{
    static THD_WORKING_AREA(ball_sense_thd_wa, BALL_SENSE_STACKSIZE);
    chThdCreateStatic(ball_sense_thd_wa, sizeof(ball_sense_thd_wa),
                      BALL_SENSE_PRIO, ball_sense_thd, NULL);
}
