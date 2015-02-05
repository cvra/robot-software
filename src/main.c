#include <ch.h>
#include <hal.h>
#include <chprintf.h>

BaseSequentialStream* stdout;

int main(void) {
    halInit();
    chSysInit();

    sdStart(&SD3, NULL);
    stdout = (BaseSequentialStream*)&SD3;

    chprintf(stdout, "boot\n");

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
