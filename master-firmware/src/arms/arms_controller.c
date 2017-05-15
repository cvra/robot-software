#include <ch.h>
#include <hal.h>
#include "priorities.h"
#include "main.h"
#include "config.h"

#include "can/bus_enumerator.h"
#include "can/motor_manager.h"
#include "robot_helpers/motor_helpers.h"
#include "base/base_controller.h"
#include "can/hand_driver.h"

#include "arms_controller.h"


#define ARMS_CONTROLLER_STACKSIZE 4096


scara_t left_arm;
scara_t right_arm;

hand_t left_hand;
hand_t right_hand;


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
    /* Configure left arm */
    scara_init(&left_arm);
    static cvra_arm_motor_t left_z = {.id = "left-z", .direction = 1, .index = 0};
    static cvra_arm_motor_t left_shoulder = {.id = "left-shoulder", .direction = -1, .index = 0};
    static cvra_arm_motor_t left_elbow = {.id = "left-elbow", .direction = -1, .index = 0};
    static cvra_arm_wrist_t left_wrist = {
        .up = "left-wrist-up", .down = "left-wrist-down",
        .up_direction = 1, .down_direction = -1,
        .heading_ratio = 6.f, .pitch_ratio = 3.f,
        .heading_index = 0, .pitch_index = 0,
    };

    scara_set_z_callbacks(&left_arm, set_motor_position, get_motor_position, &left_z);
    scara_set_shoulder_callbacks(&left_arm, set_motor_position, set_motor_velocity, get_motor_position, &left_shoulder);
    scara_set_elbow_callbacks(&left_arm, set_motor_position, set_motor_velocity, get_motor_position, &left_elbow);
    scara_set_wrist_callbacks(&left_arm, set_wrist_position, set_wrist_velocity, get_wrist_position, &left_wrist);

    scara_set_related_robot_pos(&left_arm, &robot.pos);

    scara_set_physical_parameters(&left_arm,
                                  config_get_scalar("master/arms/upperarm_length"),
                                  config_get_scalar("master/arms/forearm_length"),
                                  config_get_scalar("master/arms/wrist_to_hand_length"));

    scara_set_offset(&left_arm, config_get_scalar("master/arms/left/offset_x"),
                     config_get_scalar("master/arms/left/offset_y"),
                     config_get_scalar("master/arms/left/offset_a"));

    /* Configure right arm */
    scara_init(&right_arm);
    static cvra_arm_motor_t right_z = {.id = "right-z", .direction = 1, .index = 0};
    static cvra_arm_motor_t right_shoulder = {.id = "right-shoulder", .direction = -1, .index = 0};
    static cvra_arm_motor_t right_elbow = {.id = "right-elbow", .direction = -1, .index = 0};
    // static cvra_arm_motor_t right_wrist = {.id = "right-wrist", .direction = 1, .index = 0};

    scara_set_z_callbacks(&right_arm, set_motor_position, get_motor_position, &right_z);
    scara_set_shoulder_callbacks(&right_arm, set_motor_position, set_motor_velocity, get_motor_position, &right_shoulder);
    scara_set_elbow_callbacks(&right_arm, set_motor_position, set_motor_velocity, get_motor_position, &right_elbow);
    // scara_set_wrist_callbacks(&right_arm, set_motor_position, set_motor_velocity, get_motor_position, &right_wrist);

    scara_set_related_robot_pos(&right_arm, &robot.pos);

    scara_set_physical_parameters(&right_arm,
                                  config_get_scalar("master/arms/upperarm_length"),
                                  config_get_scalar("master/arms/forearm_length"),
                                  config_get_scalar("master/arms/wrist_to_hand_length"));

    scara_set_offset(&right_arm, config_get_scalar("master/arms/right/offset_x"),
                     config_get_scalar("master/arms/right/offset_y"),
                     config_get_scalar("master/arms/right/offset_a"));

    /* Configure left hand */
    hand_init(&left_hand);
    hand_set_fingers_callbacks(&left_hand, hand_driver_set_left_fingers);

    /* Configure right hand */
    hand_init(&right_hand);
    hand_set_fingers_callbacks(&right_hand, hand_driver_set_right_fingers);
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

    kp = parameter_scalar_get(parameter_find(ns, "heading/kp"));
    ki = parameter_scalar_get(parameter_find(ns, "heading/ki"));
    kd = parameter_scalar_get(parameter_find(ns, "heading/kd"));
    ilim = parameter_scalar_get(parameter_find(ns, "heading/ilimit"));
    pid_set_gains(&arm->heading_pid, kp, ki, kd);
    pid_set_integral_limit(&arm->heading_pid, ilim);

    kp = parameter_scalar_get(parameter_find(ns, "pitch/kp"));
    ki = parameter_scalar_get(parameter_find(ns, "pitch/ki"));
    kd = parameter_scalar_get(parameter_find(ns, "pitch/kd"));
    ilim = parameter_scalar_get(parameter_find(ns, "pitch/ilimit"));
    pid_set_gains(&arm->pitch_pid, kp, ki, kd);
    pid_set_integral_limit(&arm->pitch_pid, ilim);
}

static THD_FUNCTION(arms_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t *left_arm_control_params = parameter_namespace_find(&master_config,
                                                                              "left_arm/control");
    parameter_namespace_t *right_arm_control_params = parameter_namespace_find(&master_config,
                                                                               "right_arm/control");

    NOTICE("Start arm control");
    while (true) {
        if (parameter_namespace_contains_changed(left_arm_control_params)) {
            arms_update_controller_gains(left_arm_control_params, &left_arm);
        }
        if (parameter_namespace_contains_changed(right_arm_control_params)) {
            arms_update_controller_gains(right_arm_control_params, &right_arm);
        }

        scara_manage(&left_arm);
        // scara_manage(&right_arm);

        if (left_arm.kinematics_solution_count == 0) {
            palSetPad(GPIOF, GPIOF_LED_ERROR);
        } else {
            palClearPad(GPIOF, GPIOF_LED_ERROR);
        }

        // if (right_arm.kinematics_solution_count == 0) {
        //     palSetPad(GPIOF, GPIOF_LED_POWER_ERROR);
        // } else {
        //     palClearPad(GPIOF, GPIOF_LED_POWER_ERROR);
        // }

        hand_manage(&left_hand);
        hand_manage(&right_hand);

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

void arms_wrist_auto_index(cvra_arm_wrist_t** wrists, float* heading_speeds, float* pitch_speeds, size_t num_wrists)
{
    /* Fetch all motor drivers */
    motor_driver_t* drivers_up[num_wrists];
    motor_driver_t* drivers_down[num_wrists];
    for (size_t i = 0; i < num_wrists; i++) {
        drivers_up[i] = get_motor_driver(wrists[i]->up);
        drivers_down[i] = get_motor_driver(wrists[i]->down);
    }

    /* Enable index stream over CAN */
    for (size_t i = 0; i < num_wrists; i++) {
        set_index_stream_frequency(wrists[i]->up, 10);
        set_index_stream_frequency(wrists[i]->down, 10);
    }
#ifndef TESTS
    // TODO: Wait for an acknowledge that it was changed on the motor board, instead of waiting
    chThdSleepSeconds(1);
#endif

    bool wrist_finished[num_wrists];
    uint32_t index_counts[num_wrists];
    size_t num_finished;
    float heading_indexes[num_wrists];
    float pitch_indexes[num_wrists];

    /*** Index pitch ***/

    /* Start moving in forward direction */
    for (size_t i = 0; i < num_wrists; i++) {
        wrist_finished[i] = false;
        pitch_indexes[i] = 0.f;
        index_counts[i] = drivers_down[i]->stream.value_stream_index_update_count;
        set_wrist_velocity(wrists[i], 0, - pitch_speeds[i]);
        NOTICE("Moving %s / %s axis...", wrists[i]->up, wrists[i]->down);
    }

    /* Wait for all wrists to reach indexes */
    num_finished = 0;
    while (num_finished != num_wrists) {
        for (size_t i = 0; i < num_wrists; i++) {
            if (!wrist_finished[i] && drivers_down[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_wrist_velocity(wrists[i], 0, 0);

                /* Update index */
                float heading, pitch;
                get_wrist_position(wrists[i], &heading, &pitch);
                pitch_indexes[i] += pitch;

                /* Mark motor as done */
                wrist_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /* Move away from index positions */
    for (size_t i = 0; i < num_wrists; i++) {
        set_wrist_velocity(wrists[i], 0, - pitch_speeds[i]);
    }
#ifndef TESTS
    chThdSleepMilliseconds(1000);
#endif
    for (size_t i = 0; i < num_wrists; i++) {
        set_wrist_velocity(wrists[i], 0, 0);
    }

    /* Start moving in backward direction */
    for (size_t i = 0; i < num_wrists; i++) {
        wrist_finished[i] = false;
        index_counts[i] = drivers_down[i]->stream.value_stream_index_update_count;
        set_wrist_velocity(wrists[i], 0, pitch_speeds[i]);
        NOTICE("Moving %s / %s axis...", wrists[i]->up, wrists[i]->down);
    }

    /* Wait for all wrists to reach indexes */
    num_finished = 0;
    while (num_finished != num_wrists) {
        for (size_t i = 0; i < num_wrists; i++) {
            if (!wrist_finished[i] && drivers_down[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_wrist_velocity(wrists[i], 0, 0);

                /* Update index */
                float heading, pitch;
                get_wrist_position(wrists[i], &heading, &pitch);
                pitch_indexes[i] += pitch;

                /* Mark motor as done */
                wrist_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /*** Heading pitch ***/

    /* Start moving in forward direction */
    for (size_t i = 0; i < num_wrists; i++) {
        wrist_finished[i] = false;
        heading_indexes[i] = 0.f;
        index_counts[i] = drivers_up[i]->stream.value_stream_index_update_count;
        set_wrist_velocity(wrists[i], - heading_speeds[i], 0);
        NOTICE("Moving %s / %s axis...", wrists[i]->up, wrists[i]->down);
    }

    /* Wait for all wrists to reach indexes */
    num_finished = 0;
    while (num_finished != num_wrists) {
        for (size_t i = 0; i < num_wrists; i++) {
            if (!wrist_finished[i] && drivers_up[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_wrist_velocity(wrists[i], 0, 0);

                /* Update index */
                float heading, pitch;
                get_wrist_position(wrists[i], &heading, &pitch);
                heading_indexes[i] += heading;

                /* Mark motor as done */
                wrist_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /* Move away from index positions */
    for (size_t i = 0; i < num_wrists; i++) {
        set_wrist_velocity(wrists[i], - heading_speeds[i], 0);
    }
#ifndef TESTS
    chThdSleepMilliseconds(1000);
#endif
    for (size_t i = 0; i < num_wrists; i++) {
        set_wrist_velocity(wrists[i], 0, 0);
    }

    /* Start moving in backward direction */
    for (size_t i = 0; i < num_wrists; i++) {
        wrist_finished[i] = false;
        index_counts[i] = drivers_up[i]->stream.value_stream_index_update_count;
        set_wrist_velocity(wrists[i], heading_speeds[i], 0);
        NOTICE("Moving %s / %s axis...", wrists[i]->up, wrists[i]->down);
    }

    /* Wait for all wrists to reach indexes */
    num_finished = 0;
    while (num_finished != num_wrists) {
        for (size_t i = 0; i < num_wrists; i++) {
            if (!wrist_finished[i] && drivers_up[i]->stream.value_stream_index_update_count != index_counts[i]) {
                /* Stop motor */
                set_wrist_velocity(wrists[i], 0, 0);

                /* Update index */
                float heading, pitch;
                get_wrist_position(wrists[i], &heading, &pitch);
                heading_indexes[i] += heading;

                /* Mark motor as done */
                wrist_finished[i] = true;
                num_finished++;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(50);
#endif
    }

    /* Compute index */
    for (size_t i = 0; i < num_wrists; i++) {
        wrists[i]->pitch_index = 0.5 * pitch_indexes[i];
        wrists[i]->heading_index = 0.5 * heading_indexes[i];
        NOTICE("Wrist %d  pitch index %.3f heading index %.3f", i, wrists[i]->pitch_index, wrists[i]->heading_index);
    }

    /* Disable index stream over CAN */
    for (size_t i = 0; i < num_wrists; i++) {
        set_index_stream_frequency(wrists[i]->up, 0);
        set_index_stream_frequency(wrists[i]->down, 0);
    }
    NOTICE("Disabled motor index streams");
}
