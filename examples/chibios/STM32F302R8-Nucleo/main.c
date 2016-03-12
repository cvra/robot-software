#include "ch.h"
#include "hal.h"

static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    chRegSetThreadName("blinker");
    while (true) {
        palClearPad(GPIOB, GPIOB_LED_GREEN);
        chThdSleepMilliseconds(500);
        palSetPad(GPIOB, GPIOB_LED_GREEN);
        chThdSleepMilliseconds(500);
    }
}

int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    while (true) {
        if (!palReadPad(GPIOC, GPIOC_BUTTON))
            TestThread(&SD2);
        chThdSleepMilliseconds(500);
    }
}
