#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include "usbconf.h"

int main(void)
{
    halInit();
    chSysInit();

    usb_start();
    chprintf((BaseSequentialStream *)&SDU1, "CVRA UWB Beacon booting...\r\n");

    while(true) {
        chprintf((BaseSequentialStream *)&SDU1, "Hello from USB\r\n");

        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(1000);
    }
}
