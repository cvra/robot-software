#include "ch.h"
#include "hal.h"

int main(void) {
    halInit();
    chSysInit();

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
