#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "usbconf.h"
#include "cmd.h"
#include "bootloader_config.h"

int main(void)
{
    halInit();
    chSysInit();

    sdStart(&SD2, NULL);
    chprintf((BaseSequentialStream *)&SD2, "boot\r\n");

    bootloader_config_t boot_config;

    if (!config_get(&boot_config)) {
        chSysHalt("Could not read config!");
    }

    usb_start(boot_config.ID);

    shell_start((BaseSequentialStream *)&SDU1);

    while(true) {
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(1000);
    }
}
