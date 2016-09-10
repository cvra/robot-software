#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "usbconf.h"
#include "cmd.h"

int main(void)
{
    halInit();
    chSysInit();

    sdStart(&SD2, NULL);
    chprintf((BaseSequentialStream *)&SD2, "boot\r\n");

    usb_start();

    shell_start((BaseSequentialStream *)&SDU1);

    while(true) {
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(1000);
    }
}
