#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <parameter_flash_storage/parameter_flash_storage.h>
#include <trace/trace.h>
#include <error/error.h>

#include "main.h"
#include "usbconf.h"
#include "cmd.h"
#include "exti.h"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "./uavcan/uavcan_node.h"
#include "ranging_thread.h"
#include "state_estimation_thread.h"
#include "anchor_position_cache.h"
#include "log.h"

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

    trace_init();
    trace_enable();
}

int main(void)
{
    /* Starts USB, this takes about 1 second, as we have to disconnect and
     * reconnect the device. */
    usb_start(42);
    log_init();

    NOTICE("boot");

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    parameter_namespace_declare(&parameter_root, NULL, NULL);

    blink_start();
    exti_start();
    imu_start(); // disabled so that the EKF does not run
    ahrs_start();
    ranging_start();
    anchor_position_cache_start();
    state_estimation_start();

    //uavcan_node_start(42, "uwb-beacon");

    shell_start((BaseSequentialStream*)&SDU1);

    /* All services should be initialized by now, we can load the config. */
    chThdSleepMilliseconds(1000);
    parameter_flash_storage_load(&parameter_root, &_config_start);

    while (true) {
        chThdSleepMilliseconds(1000);
    }
}

static void blink_thd(void* p)
{
    (void)p;

    while (1) {
        board_led_toggle(BOARD_LED_STATUS);
        chThdSleepMilliseconds(200);
    }
}

static void blink_start(void)
{
    static THD_WORKING_AREA(blink_wa, 256);
    chThdCreateStatic(blink_wa, sizeof(blink_wa), LOWPRIO, blink_thd, NULL);
}
