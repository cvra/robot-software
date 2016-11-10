#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "bootloader_config.h"

int main(void)
{
    halInit();
    chSysInit();

    bootloader_config_t cfg;

    if (!config_get(&cfg)) {
        chSysHalt("Cannot load config");
    }

    uavcan_start(cfg.ID, cfg.board_name);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
