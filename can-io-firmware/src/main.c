#include <ch.h>
#include <hal.h>
#include <uavcan/node.h>
#include "bootloader_config.h"
#include <error/error.h>
#include "debug.h"
#include "servo.h"
#include "main.h"

THD_FUNCTION(blinker, arg)
{
    (void)arg;
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

void _unhandled_exception(void)
{
    chSysHalt("unhandled exception");

    while (true) {
        /* wait forever */
    }
}

bootloader_config_t config;

int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    blinker_start();

    servo_init();
    servo_start();

    if (!config_get(&config)) {
        chSysHalt("Cannot load config");
    }

    NOTICE("Board name=\"%s\", ID=%d", config.board_name, config.ID);

    // Never returns
    uavcan_start(config.ID, config.board_name);

    return 0;
}
