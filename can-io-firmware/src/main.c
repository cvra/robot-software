#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "bootloader_config.h"

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
    chThdCreateStatic(blinker_wa, sizeof(blinker_wa), LOWPRIO,blinker, NULL);
}


int main(void)
{
    halInit();
    chSysInit();

    blinker_start();

    bootloader_config_t cfg;

    if (!config_get(&cfg)) {
        chSysHalt("Cannot load config");
    }

    // Never returns
    uavcan_start(cfg.ID, cfg.board_name);
}
