#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "main.h"
#include "usbconf.h"
#include "cmd.h"
#include "bootloader_config.h"
#include "exti.h"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "uavcan/uavcan_node.h"
#include "decawave_interface.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root;

static void blink_start(void);

/** Late init hook, called before c++ static constructors. */
void __late_init(void)
{
    /* C++ Static initializer requires working chibios. */
    halInit();
    chSysInit();
}


int main(void)
{
    sdStart(&SD2, NULL);
    chprintf((BaseSequentialStream *)&SD2, "boot\r\n");

    bootloader_config_t boot_config;

    if (!config_get(&boot_config)) {
        chSysHalt("Could not read config!");
    }

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);

    usb_start(boot_config.ID);
    shell_start((BaseSequentialStream *)&SDU1);

    blink_start();
    exti_start();
    imu_start();
    ahrs_start();
    decawave_start();
    uavcan_node_start(boot_config.ID, boot_config.board_name);

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
