#include <ch.h>
#include <hal.h>

#include <error/error.h>

#include "ball_sense.h"

#define BALL_SENSE_STACKSIZE 512

static inline bool ball_sense_read(void)
{
    return palReadPad(GPIOG, GPIOG_PIN9);
}

static THD_FUNCTION(ball_sense_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    NOTICE("Ball sensor ready to count balls");

    while (true) {
        bool sense = ball_sense_read();

        chThdSleepMilliseconds(1);
    }
}

void ball_sense_start(void)
{
    static THD_WORKING_AREA(ball_sense_thd_wa, BALL_SENSE_STACKSIZE);
    chThdCreateStatic(ball_sense_thd_wa, sizeof(ball_sense_thd_wa),
                      BALL_SENSE_PRIO, ball_sense_thd, NULL);
}
