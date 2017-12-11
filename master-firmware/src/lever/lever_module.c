#include "lever_module.h"

#include "arms/cvra_arm_motors.h"
#include "error/error.h"
#include "pca9685_pwm.h"

#include <ch.h>
#include "priorities.h"
#include "main.h"
#include "config.h"

#define LEVER_MODULE_STACKSIZE 256

lever_t main_lever;

static void set_servo(void* servo, float pos)
{
    unsigned int* servo_nb = (unsigned int *)servo;
    pca9685_pwm_set_pulse_width(*servo_nb, pos);
}

static void lever_update_settings(lever_t* lever, parameter_namespace_t* ns)
{
    lever_set_servo_range(lever, 0.0009, 0.0022);
}

static void lever_module_init(lever_t* lever, parameter_namespace_t* ns)
{
    /* Lever init */
    lever_init(lever);
    lever_update_settings(lever, ns);

    static unsigned int lever_servo_nb = 0;
    lever_set_callbacks(lever, set_servo, &lever_servo_nb);

    static cvra_arm_motor_t lever_pump1 = {.id = "lever-pump-1", .direction = 0, .index = 0};
    static cvra_arm_motor_t lever_pump2 = {.id = "lever-pump-2", .direction = 0, .index = 0};
    lever_pump_set_callbacks(lever, set_motor_voltage, &lever_pump1, &lever_pump2);
}

static THD_FUNCTION(lever_module_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* lever_params = parameter_namespace_find(&master_config, "lever");

    NOTICE("Starting lever module");

    lever_module_init(&main_lever, lever_params);

    while (true) {
        if (parameter_namespace_contains_changed(lever_params)) {
            lever_update_settings(&main_lever, lever_params);
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
