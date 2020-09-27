#include <ch.h>
#include <hal.h>
#include <uavcan/node.h>
#include "analog_input.h"
#include "bootloader_config.h"
#include <error/error.h>
#include "pressure_sensor_interface.h"
#include "debug.h"
#include "servo.h"
#include "pump.h"
#include "main.h"

THD_FUNCTION(blinker, arg)
{
    (void)arg;
    while (1) {
        board_set_led(true);
        chThdSleepMilliseconds(100);
        board_set_led(false);
        chThdSleepMilliseconds(100);
    }
}

void _unhandled_exception(void)
{
    chSysHalt("unhandled exception");

    while (true) {
        /* wait forever */
    }
}

bootloader_config_t config;

int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    board_reset_pressure_sensors();

    analog_start();

    servo_start();
    mpr_start();
    pump_init();

    if (!config_get(&config)) {
        uavcan_set_node_is_ok(false);
    }

    NOTICE("Board name=\"%s\", ID=%d", config.board_name, config.ID);

    uavcan_start(config.ID, config.board_name);

    // Never returns
    blinker(NULL);

    return 0;
}
