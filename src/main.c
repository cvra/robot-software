#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "usbconf.h"
#include "cmd.h"

int main(void)
{
    halInit();
    chSysInit();

    usb_start();
    chprintf((BaseSequentialStream *)&SDU1, "CVRA UWB Beacon booting...\r\n");

    shell_start((BaseSequentialStream *)&SDU1);

    while(true) {
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(1000);
    }
}
