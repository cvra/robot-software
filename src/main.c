#include "ch.h"
#include "hal.h"

int main(void) {
    halInit();
    chSysInit();

    while (1) {
        chThdSleepMilliseconds(1000);
    }
}
