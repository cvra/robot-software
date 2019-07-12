#include <ch.h>
#include <hal.h>
#include <uavcan/node.h>
#include "bootloader_config.h"
#include <error/error.h>
#include "debug.h"
#include "main.h"

#define STATUS_LED PAL_LINE(GPIOA, GPIOA_SERVO0)

void led_clear(void)
{
    palSetLine(STATUS_LED);
    palSetPad(GPIOA, GPIOA_LED);
}

void led_set(void)
{
    palClearLine(STATUS_LED);
    palClearPad(GPIOA, GPIOA_LED);
}

THD_FUNCTION(blinker, arg)
{
    palSetLineMode(STATUS_LED, PAL_MODE_OUTPUT_PUSHPULL);
    (void)arg;
    while (1) {
        if (electron_state == INIT) {
            led_clear();
            chThdSleepMilliseconds(40);
            led_set();
            chThdSleepMilliseconds(40);

            led_clear();
            chThdSleepMilliseconds(40);
            led_set();
            chThdSleepMilliseconds(40);

            led_clear();
            chThdSleepMilliseconds(40);
            led_set();
            chThdSleepMilliseconds(40);

            led_clear();
            chThdSleepMilliseconds(40);
            led_set();

            chThdSleepMilliseconds(720);
        } else if (electron_state == READY) {
            led_clear();
            chThdSleepMilliseconds(80);
            led_set();
            chThdSleepMilliseconds(80);
            led_clear();
            chThdSleepMilliseconds(80);
            led_set();
            chThdSleepMilliseconds(760);
        } else if (electron_state == RUNNING) {
            led_set();
            chThdSleepMilliseconds(100);
            led_clear();
            chThdSleepMilliseconds(100);
        } else {
            led_set();
            chThdSleepMilliseconds(1000);
            led_clear();
            chThdSleepMilliseconds(1000);
        }
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

    if (!config_get(&config)) {
        chSysHalt("Cannot load config");
    }

    NOTICE("Board name=\"%s\", ID=%d", config.board_name, config.ID);

    // Never returns
    uavcan_start(config.ID, config.board_name);

    return 0;
}
