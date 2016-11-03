#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"

int main(void)
{
    halInit();
    chSysInit();

    uavcan_start();

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
