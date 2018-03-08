#include "lever_module.h"

#include "arms/cvra_arm_motors.h"
#include "error/error.h"

#include "can/can_io_driver.h"

#include <ch.h>
#include "priorities.h"
#include "main.h"
#include "config.h"

#define LEVER_MODULE_STACKSIZE 512

lever_t right_lever, left_lever;
char* right_lever_name = "right-lever";
char* left_lever_name = "left-lever";

static void set_servo(void* lever, float pos)
{
    char* name = (char*)lever;
    can_io_set_pwm(name, 0, pos);
}

static void lever_update_settings(lever_t* lever, parameter_namespace_t* ns)
{
    float deployed, retracted;
    deployed = parameter_scalar_get(parameter_find(ns, "servo/deployed"));
    retracted = parameter_scalar_get(parameter_find(ns, "servo/retracted"));

    lever_set_servo_range(lever, retracted, deployed);
}

static THD_FUNCTION(lever_module_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* right_lever_params = parameter_namespace_find(&master_config, "lever/right");
    parameter_namespace_t* left_lever_params = parameter_namespace_find(&master_config, "lever/left");

    NOTICE("Starting lever module");

    /* Right lever */
    lever_init(&right_lever);
    lever_update_settings(&right_lever, right_lever_params);
    lever_set_callbacks(&right_lever, set_servo, right_lever_name);

    static cvra_arm_motor_t right_pump1 = {.id = "right-pump-1", .direction = 0, .index = 0};
    static cvra_arm_motor_t right_pump2 = {.id = "right-pump-2", .direction = 0, .index = 0};
    lever_pump_set_callbacks(&right_lever, set_motor_voltage, &right_pump1, &right_pump2);

    /* Left lever */
    lever_init(&left_lever);
    lever_update_settings(&left_lever, left_lever_params);
    lever_set_callbacks(&left_lever, set_servo, left_lever_name);

    static cvra_arm_motor_t left_pump1 = {.id = "left-pump-1", .direction = 0, .index = 0};
    static cvra_arm_motor_t left_pump2 = {.id = "left-pump-2", .direction = 0, .index = 0};
    lever_pump_set_callbacks(&left_lever, set_motor_voltage, &left_pump1, &left_pump2);

    while (true) {
        if (parameter_namespace_contains_changed(right_lever_params)) {
            lever_update_settings(&right_lever, right_lever_params);
        }
        if (parameter_namespace_contains_changed(left_lever_params)) {
            lever_update_settings(&left_lever, left_lever_params);
        }

        chThdSleepMilliseconds(1000 / LEVER_FREQUENCY);
    }
}

void lever_module_start(void)
{
    static THD_WORKING_AREA(lever_module_thd_wa, LEVER_MODULE_STACKSIZE);
    chThdCreateStatic(lever_module_thd_wa,
                      sizeof(lever_module_thd_wa),
                      LEVER_MODULE_PRIO,
                      lever_module_thd,
                      NULL);
}
