#include <ch.h>
#include <hal.h>
#include "priorities.h"
#include "main.h"
#include "config.h"

#include "can/bus_enumerator.h"
#include "can/motor_manager.h"
#include "robot_helpers/motor_helpers.h"
#include "base/base_controller.h"

#include "arms_controller.h"


#define ARMS_CONTROLLER_STACKSIZE 4096


scara_t main_arm;


static void set_index_stream_frequency(char* motor, float freq)
{
    motor_driver_t* driver = get_motor_driver(motor);

    if (parameter_defined(&(driver->config.index_stream))) {
        parameter_scalar_set(&(driver->config.index_stream), freq);
    } else {
        ERROR("Undefined motor %s", motor);
    }
}

void arms_init(void)
{
    /* Configure main arm */
    scara_init(&main_arm);
    static cvra_arm_motor_t z_joint = {.id = "z-joint", .direction = 1, .index = 0};
    static cvra_arm_motor_t shoulder_joint = {.id = "shoulder-joint", .direction = -1, .index = 0};
    static cvra_arm_motor_t elbow_joint = {.id = "elbow-joint", .direction = -1, .index = 0};

    joint_set_callbacks(&(main_arm.z_joint), set_motor_position, set_motor_velocity, get_motor_position, &z_joint);
    joint_set_callbacks(&(main_arm.elbow_joint), set_motor_position, set_motor_velocity, get_motor_position, &elbow_joint);
    joint_set_callbacks(&(main_arm.shoulder_joint), set_motor_position, set_motor_velocity, get_motor_position, &shoulder_joint);

    scara_set_related_robot_pos(&main_arm, &robot.pos);

    scara_set_physical_parameters(&main_arm,
                                  config_get_scalar("master/arms/upperarm_length"),
                                  config_get_scalar("master/arms/forearm_length"));

    scara_set_offset(&main_arm, config_get_scalar("master/arms/main_arm/offset_x"),
                     config_get_scalar("master/arms/main_arm/offset_y"),
                     config_get_scalar("master/arms/main_arm/offset_a"));
}

float arms_motor_auto_index(const char* motor_name, int motor_dir, float motor_speed)
{
    motor_driver_t* motor = bus_enumerator_get_driver(motor_manager.bus_enumerator, motor_name);
    if (motor == NULL) {
        ERROR("Motor \"%s\" doesn't exist", motor_name);
    }

    return motor_auto_index(motor, motor_dir, motor_speed);
}

static void arms_update_controller_gains(parameter_namespace_t* ns, scara_t* arm)
{
    float kp, ki, kd, ilim;

    kp = parameter_scalar_get(parameter_find(ns, "x/kp"));
    ki = parameter_scalar_get(parameter_find(ns, "x/ki"));
    kd = parameter_scalar_get(parameter_find(ns, "x/kd"));
    ilim = parameter_scalar_get(parameter_find(ns, "x/ilimit"));
    pid_set_gains(&arm->x_pid, kp, ki, kd);
    pid_set_integral_limit(&arm->x_pid, ilim);

    kp = parameter_scalar_get(parameter_find(ns, "y/kp"));
    ki = parameter_scalar_get(parameter_find(ns, "y/ki"));
    kd = parameter_scalar_get(parameter_find(ns, "y/kd"));
    ilim = parameter_scalar_get(parameter_find(ns, "y/ilimit"));
    pid_set_gains(&arm->y_pid, kp, ki, kd);
    pid_set_integral_limit(&arm->y_pid, ilim);
}

static THD_FUNCTION(arms_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t *main_arm_control_params = parameter_namespace_find(&master_config, "main_arm/control");

    NOTICE("Start arm control");
    while (true) {
        if (parameter_namespace_contains_changed(main_arm_control_params)) {
            arms_update_controller_gains(main_arm_control_params, &main_arm);
        }

        scara_manage(&main_arm);

        chThdSleepMilliseconds(1000 / ARMS_FREQUENCY);
    }
}

void arms_controller_start(void)
{
    static THD_WORKING_AREA(arms_ctrl_thd_wa, ARMS_CONTROLLER_STACKSIZE);
    chThdCreateStatic(arms_ctrl_thd_wa,
                      sizeof(arms_ctrl_thd_wa),
                      ARMS_CONTROLLER_PRIO,
                      arms_ctrl_thd,
                      NULL);
}

void arms_auto_index(cvra_arm_motor_t** motors, float* motor_speeds, size_t num_motors)
{
    /* Fetch all motor drivers */
    motor_driver_t* drivers[num_motors];
    for (size_t i = 0; i < num_motors; i++) {
        drivers[i] = get_motor_driver(motors[i]->id);
    }

    /* Enable index stream over CAN */
    for (size_t i = 0; i < num_motors; i++) {
        set_index_stream_frequency(motors[i]->id, 10);
    }
#ifndef TESTS
    // TODO: Wait for an acknowledge that it was changed on the motor board, instead of waiting
    chThdSleepSeconds(1);
#endif

    /* Start moving in forward direction */
    bool motor_finished[num_motors];
    uint32_t index_counts[num_motors];
    for (size_t i = 0; i < num_motors; i++) {
        motor_finished[i] = false;
        motors[i]->index = 0.f;
        index_counts[i] = drivers[i]->stream.value_stream_index_update_count;
        set_motor_velocity(motors[i], - motors[i]->direction * motor_speeds[i]);
        NOTICE("Moving %s axis...", motors[i]->id);
    }

    /* Wait for all motors to reach indexes */
    size_t num_finished = 0;
    while (num_finished != num_motors) {
        for (size_t i = 0; i < num_motors; i++) {
            if (!motor_finished[i] && drivers[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_motor_velocity(motors[i], 0);

                /* Update index */
                motors[i]->index += motor_driver_get_and_clear_stream_value(drivers[i], MOTOR_STREAM_INDEX);

                /* Mark motor as done */
                motor_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /* Move away from index positions */
    for (size_t i = 0; i < num_motors; i++) {
        set_motor_velocity(motors[i], - motors[i]->direction * motor_speeds[i]);
    }
#ifndef TESTS
    chThdSleepMilliseconds(1000);
#endif
    for (size_t i = 0; i < num_motors; i++) {
        set_motor_velocity(motors[i], 0);
    }

    /* Start moving in backward direction */
    for (size_t i = 0; i < num_motors; i++) {
        motor_finished[i] = false;
        index_counts[i] = drivers[i]->stream.value_stream_index_update_count;
        set_motor_velocity(motors[i], motors[i]->direction * motor_speeds[i]);
        NOTICE("Moving %s axis...", motors[i]->id);
    }

    /* Wait for all motors to reach indexes */
    num_finished = 0;
    while (num_finished != num_motors) {
        for (size_t i = 0; i < num_motors; i++) {
            if (!motor_finished[i] && drivers[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_motor_velocity(motors[i], 0);

                /* Update index */
                motors[i]->index += motor_driver_get_and_clear_stream_value(drivers[i], MOTOR_STREAM_INDEX);

                /* Mark motor as done */
                motor_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /* Compute index */
    for (size_t i = 0; i < num_motors; i++) {
        motors[i]->index *= 0.5;
        NOTICE("Motor %s index %.3f", motors[i]->id, motors[i]->index);
    }

    /* Disable index stream over CAN */
    for (size_t i = 0; i < num_motors; i++) {
        set_index_stream_frequency(motors[i]->id, 0);
    }
    NOTICE("Disabled motor index streams");
}
