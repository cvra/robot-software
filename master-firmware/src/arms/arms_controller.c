#include <ch.h>
#include "priorities.h"
#include "main.h"
#include "config.h"

#include "can/bus_enumerator.h"
#include "can/motor_manager.h"
#include "robot_helpers/motor_helpers.h"
#include "base/base_controller.h"

#include "cvra_arm_motors.h"
#include "arms_controller.h"


#define ARMS_CONTROLLER_STACKSIZE 1024


scara_t left_arm;
scara_t right_arm;


void arms_init(void)
{
    /* Configure left arm */
    scara_init(&left_arm);
    static cvra_arm_motor_t left_z = {.m = &motor_manager, .direction = 1, .index = 0};
    static cvra_arm_motor_t left_shoulder = {.m = &motor_manager, .direction = -1, .index = 0};
    static cvra_arm_motor_t left_elbow = {.m = &motor_manager, .direction = -1, .index = 0};

    scara_set_z_callbacks(&left_arm, set_left_z_position, get_left_z_position, &left_z);
    scara_set_shoulder_callbacks(&left_arm, set_left_shoulder_position, get_left_shoulder_position, &left_shoulder);
    scara_set_elbow_callbacks(&left_arm, set_left_elbow_position, get_left_elbow_position, &left_elbow);

    scara_set_related_robot_pos(&left_arm, &robot.pos);

    scara_set_physical_parameters(&left_arm,
        config_get_scalar("master/arms/upperarm_length"),
        config_get_scalar("master/arms/forearm_length"));

    scara_set_offset(&left_arm, config_get_scalar("master/arms/left/offset_x"),
        config_get_scalar("master/arms/left/offset_y"),
        config_get_scalar("master/arms/left/offset_a"));

    /* Configure right arm */
    scara_init(&right_arm);
    static cvra_arm_motor_t right_z = {.m = &motor_manager, .direction = 1, .index = 0};
    static cvra_arm_motor_t right_shoulder = {.m = &motor_manager, .direction = -1, .index = 0};
    static cvra_arm_motor_t right_elbow = {.m = &motor_manager, .direction = -1, .index = 0};

    scara_set_z_callbacks(&right_arm, set_right_z_position, get_right_z_position, &right_z);
    scara_set_shoulder_callbacks(&right_arm, set_right_shoulder_position, get_right_shoulder_position, &right_shoulder);
    scara_set_elbow_callbacks(&right_arm, set_right_elbow_position, get_right_elbow_position, &right_elbow);

    scara_set_related_robot_pos(&right_arm, &robot.pos);

    scara_set_physical_parameters(&right_arm,
        config_get_scalar("master/arms/upperarm_length"),
        config_get_scalar("master/arms/forearm_length"));

    scara_set_offset(&right_arm, config_get_scalar("master/arms/right/offset_x"),
        config_get_scalar("master/arms/right/offset_y"),
        config_get_scalar("master/arms/right/offset_a"));
}

float arms_motor_auto_index(const char* motor_name, int motor_dir, float motor_speed)
{
    motor_driver_t* motor = bus_enumerator_get_driver(motor_manager.bus_enumerator, motor_name);
    if (motor == NULL) {
        chSysHalt("Motor doesn't exist");
    }

    return motor_auto_index(motor, motor_dir, motor_speed);
}

static THD_FUNCTION(arms_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while (true) {
        scara_manage(&left_arm);
        scara_manage(&right_arm);

        chThdSleepMilliseconds(1000 / ARMS_FREQUENCY);
    }
}

void arms_controller_start(void)
{
    static THD_WORKING_AREA(arms_ctrl_thd_wa, ARMS_CONTROLLER_STACKSIZE);
    chThdCreateStatic(arms_ctrl_thd_wa, sizeof(arms_ctrl_thd_wa), ARMS_CONTROLLER_PRIO, arms_ctrl_thd, NULL);
}

void arms_auto_index(const char** motor_names, int* motor_dirs, float* motor_speeds, size_t num_motors, float* motor_indexes)
{
    /* Fetch all motor drivers */
    motor_driver_t* motors[num_motors];
    for (size_t i = 0; i < num_motors; i++) {
        motors[i] = bus_enumerator_get_driver(motor_manager.bus_enumerator, motor_names[i]);
        if (motors[i] == NULL) {
            chSysHalt("Motor doesn't exist");
        }
    }

    /* Enable index stream over CAN */
    for (size_t i = 0; i < num_motors; i++) {
        parameter_scalar_set(&(motors[i]->config.index_stream), 10);
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
        motor_indexes[i] = 0.;
        index_counts[i] = motors[i]->stream.value_stream_index_update_count;
        motor_driver_set_velocity(motors[i], - motor_dirs[i] * motor_speeds[i]);
        NOTICE("Moving %s axis...", motor_names[i]);
    }

    /* Wait for all motors to reach indexes */
    size_t num_finished = 0;
    while(num_finished != num_motors) {
        for (size_t i = 0; i < num_motors; i++) {
            if(!motor_finished[i] && motors[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                motor_driver_set_velocity(motors[i], 0.);

                /* Update index */
                motor_indexes[i] += motor_driver_get_and_clear_stream_value(motors[i], MOTOR_STREAM_INDEX);

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
        motor_driver_set_velocity(motors[i], - motor_dirs[i] * motor_speeds[i]);
    }
#ifndef TESTS
    chThdSleepMilliseconds(1000);
#endif
    for (size_t i = 0; i < num_motors; i++) {
        motor_driver_set_velocity(motors[i], 0);
    }

    /* Start moving in backward direction */
    for (size_t i = 0; i < num_motors; i++) {
        motor_finished[i] = false;
        index_counts[i] = motors[i]->stream.value_stream_index_update_count;
        motor_driver_set_velocity(motors[i], motor_dirs[i] * motor_speeds[i]);
        NOTICE("Moving %s axis...", motor_names[i]);
    }

    /* Wait for all motors to reach indexes */
    num_finished = 0;
    while(num_finished != num_motors) {
        for (size_t i = 0; i < num_motors; i++) {
            if(!motor_finished[i] && motors[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                motor_driver_set_velocity(motors[i], 0.);

                /* Update index */
                motor_indexes[i] += motor_driver_get_and_clear_stream_value(motors[i], MOTOR_STREAM_INDEX);

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
        motor_indexes[i] *= 0.5;
    }

    /* Disable index stream over CAN */
    for (size_t i = 0; i < num_motors; i++) {
        parameter_scalar_set(&(motors[i]->config.index_stream), 0);
    }
    NOTICE("Disabled motor index streams");
}


void arms_set_motor_index(void* motor, float index)
{
    cvra_arm_motor_t *dev = (cvra_arm_motor_t*)motor;
    dev->index = index;
}
