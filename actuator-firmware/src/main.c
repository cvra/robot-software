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

THD_FUNCTION(heartbeat, arg)
{
    chRegSetThreadName("heartbeat");
    (void)arg;
    while (1) {
        board_set_led(true);
        chThdSleepMilliseconds(80);
        board_set_led(false);
        chThdSleepMilliseconds(80);
        board_set_led(true);
        chThdSleepMilliseconds(80);
        board_set_led(false);
        chThdSleepMilliseconds(760);
    }
}

static void heartbeat_start(void)
{
    static THD_WORKING_AREA(heartbeat_wa, 150);
    chThdCreateStatic(heartbeat_wa, sizeof(heartbeat_wa), LOWPRIO, heartbeat, NULL);
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

    heartbeat_start();

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

    while (1) {
        print_stack_info();
        chThdSleepMilliseconds(1000);
    }

    return 0;
}
