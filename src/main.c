#include <ch.h>
#include <hal.h>

int main(void)
{
    halInit();
    chSysInit();

    while(1) {
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(1000);
    }
}
