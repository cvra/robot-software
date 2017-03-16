#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "bootloader_config.h"
#include "error/error.h"
#include "debug.h"
#include "servo_pwm.h"

THD_FUNCTION(blinker, arg)
{
    (void) arg;
    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(100);
    }
}

static void blinker_start(void)
{
    static THD_WORKING_AREA(blinker_wa, 256);
    chThdCreateStatic(blinker_wa, sizeof(blinker_wa), LOWPRIO, blinker, NULL);
}


int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    blinker_start();

    bootloader_config_t cfg;

    servo_init();
    float pos[4] = {0, 0.25, 0.5, 1};
    servo_set(pos);

    if (!config_get(&cfg)) {
        chSysHalt("Cannot load config");
    }

    NOTICE("Board name=\"%s\", ID=%d", cfg.board_name, cfg.ID);

    // Never returns
    uavcan_start(cfg.ID, cfg.board_name);

    return 0;
}
