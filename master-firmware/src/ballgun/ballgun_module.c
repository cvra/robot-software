#include <ch.h>

#include <error/error.h>

#include <arms/cvra_arm_motors.h>
#include "config.h"
#include "main.h"
#include "pca9685_pwm.h"
#include "priorities.h"

#include "ballgun_module.h"

#define BALLGUN_MODULE_STACKSIZE 512

ballgun_t main_ballgun;

static void set_servo(void* ballgun, float pos)
{
    (void)ballgun;
    int channel = config_get_integer("master/ballgun/servo/channel");
    pca9685_pwm_set_pulse_width(channel, pos);
}

static void set_turbine(void* ballgun, float speed)
{
    (void)ballgun;
    int channel = config_get_integer("master/ballgun/turbine/channel");
    pca9685_pwm_set_pulse_width(channel, speed);
}

static void ballgun_update_settings(ballgun_t* ballgun, parameter_namespace_t* ns)
{
    float deployed = parameter_scalar_get(parameter_find(ns, "servo/deployed"));
    float retracted = parameter_scalar_get(parameter_find(ns, "servo/retracted"));
    float deployed_fully = parameter_scalar_get(parameter_find(ns, "servo/deployed_fully"));

    ballgun_set_servo_range(ballgun, retracted, deployed, deployed_fully);

    float arm = parameter_scalar_get(parameter_find(ns, "turbine/arm"));
    float charge = parameter_scalar_get(parameter_find(ns, "turbine/charge"));
    float fire = parameter_scalar_get(parameter_find(ns, "turbine/fire"));
    float slowfire = parameter_scalar_get(parameter_find(ns, "turbine/slowfire"));

    ballgun_set_turbine_range(ballgun, arm, charge, fire, slowfire);

    float acc_charge = parameter_scalar_get(parameter_find(ns, "accelerator/charge"));
    float acc_fire = parameter_scalar_get(parameter_find(ns, "accelerator/fire"));
    float acc_slowfire = parameter_scalar_get(parameter_find(ns, "accelerator/slowfire"));

    ballgun_set_accelerator_range(ballgun, acc_charge, acc_fire, acc_slowfire);
}

static THD_FUNCTION(ballgun_module_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t* main_ballgun_params = parameter_namespace_find(&master_config, "ballgun");

    ballgun_init(&main_ballgun);
    ballgun_update_settings(&main_ballgun, main_ballgun_params);
    ballgun_set_callbacks(&main_ballgun, set_servo, NULL);
    ballgun_set_turbine_callbacks(&main_ballgun, set_turbine, NULL);

    static cvra_arm_motor_t ball_accelerator = {.id = "ball-accelerator", .direction = 0, .index = 0};
    ballgun_set_accelerator_callbacks(&main_ballgun, set_motor_voltage, &ball_accelerator);

    NOTICE("Ball gun ready to shoot");

    while (true) {
        if (parameter_namespace_contains_changed(main_ballgun_params)) {
            ballgun_update_settings(&main_ballgun, main_ballgun_params);
        }

        chThdSleepMilliseconds(1000 / BALLGUN_FREQUENCY);
    }
}

void ballgun_module_start(void)
{
    static THD_WORKING_AREA(ballgun_module_thd_wa, BALLGUN_MODULE_STACKSIZE);
    chThdCreateStatic(ballgun_module_thd_wa, sizeof(ballgun_module_thd_wa),
                      BALLGUN_MODULE_PRIO, ballgun_module_thd, NULL);
}

void ball_accelerator_voltage(float voltage)
{
    static cvra_arm_motor_t ball_accelerator = {.id = "ball-accelerator", .direction = 0, .index = 0};
    set_motor_voltage(&ball_accelerator, voltage);
}
