#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "main.h"
#include "usbconf.h"
#include "cmd.h"
#include "bootloader_config.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void blink_start(void);

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

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    usb_start(boot_config.ID);

    shell_start((BaseSequentialStream *)&SDU1);

    blink_start();
    while(true) {
        chThdSleepMilliseconds(1000);
    }
}

static void blink_thd(void *p)
{
    (void) p;

    while (1) {
        palTogglePad(GPIOB, GPIOB_LED_STATUS);
        chThdSleepMilliseconds(200);
    }
}

static void blink_start(void)
{
    static THD_WORKING_AREA(blink_wa, 256);
    chThdCreateStatic(blink_wa, sizeof(blink_wa), LOWPRIO, blink_thd, NULL);
}
