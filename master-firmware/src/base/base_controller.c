#include <ch.h>
#include <math.h>

#include <error/error.h>
#include "main.h"
#include "config.h"
#include "priorities.h"
#include "robot_parameters.h"
#include "aversive_port/cvra_motors.h"
#include "trajectory_manager/trajectory_manager.h"
#include "trajectory_manager/trajectory_manager_utils.h"
#include "trajectory_manager/trajectory_manager_core.h"

#include "base_controller.h"

#define BASE_CONTROLLER_STACKSIZE    4096
#define POSITION_MANAGER_STACKSIZE   4096
#define TRAJECTORY_MANAGER_STACKSIZE 4096


struct _robot robot;


void robot_init(void)
{
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;

    /* Motors */
    static cvra_motor_t left_wheel_motor = {.m=&motor_manager, .max_velocity=10.f, .direction=1.};
    static cvra_motor_t right_wheel_motor = {.m=&motor_manager, .max_velocity=10.f, .direction=-1.};
    cvra_encoder_init();

    robot.angle_pid.divider = 100;
    robot.distance_pid.divider = 100;

    /* Robot system initialisation, encoders and PWM */
    rs_init(&robot.rs);
    rs_set_flags(&robot.rs, RS_USE_EXT);

    rs_set_left_pwm(&robot.rs, cvra_motor_left_wheel_set_velocity, &left_wheel_motor);
    rs_set_right_pwm(&robot.rs, cvra_motor_right_wheel_set_velocity, &right_wheel_motor);
    rs_set_left_ext_encoder(&robot.rs, cvra_encoder_get_left_ext, NULL, LEFT_WHEEL_CORRECTION_FACTOR);
    rs_set_right_ext_encoder(&robot.rs, cvra_encoder_get_right_ext, NULL, RIGHT_WHEEL_CORRECTION_FACTOR);

    /* Position manager */
    position_init(&robot.pos);
    position_set_related_robot_system(&robot.pos, &robot.rs); // Link pos manager to robot system

    position_set_physical_params(&robot.pos, ROBOT_EXTERNAL_TRACK_LENGTH_MM, EXTERNAL_ENCODER_TICKS_PER_MM);
    position_use_ext(&robot.pos);

    /* Base angle controller */
    pid_init(&robot.angle_pid.pid);
    quadramp_init(&robot.angle_qr);

    cs_init(&robot.angle_cs);
    cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr); // Filter acceleration
    cs_set_correct_filter(&robot.angle_cs, cvra_pid_process, &robot.angle_pid.pid);
    cs_set_process_in(&robot.angle_cs, rs_set_angle, &robot.rs); // Output on angular virtual pwm
    cs_set_process_out(&robot.angle_cs, rs_get_ext_angle, &robot.rs); // Read angular virtuan encoder
    cs_set_consign(&robot.angle_cs, 0);

    /* Base distance controller */
    pid_init(&robot.distance_pid.pid);
    quadramp_init(&robot.distance_qr);

    cs_init(&robot.distance_cs);
    cs_set_consign_filter(&robot.distance_cs, quadramp_do_filter, &robot.distance_qr); // Filter acceleration
    cs_set_correct_filter(&robot.distance_cs, cvra_pid_process, &robot.distance_pid.pid);
    cs_set_process_in(&robot.distance_cs, rs_set_distance, &robot.rs); // Output on distance virtual pwm
    cs_set_process_out(&robot.distance_cs, rs_get_ext_distance, &robot.rs); // Read distance virtuan encoder
    cs_set_consign(&robot.distance_cs, 0);

    /* Trajector manager */
    trajectory_manager_init(&robot.traj, ASSERV_FREQUENCY);
    trajectory_set_cs(&robot.traj, &robot.distance_cs, &robot.angle_cs);
    trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);

    // Distance window, angle window, angle start
    trajectory_set_windows(&robot.traj, 15., 1., 1.);

    /* Initialize blocking detection managers */
    bd_init(&robot.angle_bd, &robot.angle_cs);
    bd_init(&robot.distance_bd, &robot.distance_cs);

    /* Set obstacle inflation sizes */
    robot.robot_size = ROBOT_SIZE_X_MM;
    robot.opponent_size = DEFAULT_OPPONENT_SIZE_MM;
}


static THD_FUNCTION(base_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    parameter_namespace_t *control_params = parameter_namespace_find(&global_config, "master/aversive/control");
    while (1) {
        rs_update(&robot.rs);

        /* Control system manage */
        if (robot.mode != BOARD_MODE_SET_PWM) {
            if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_ANGLE_ONLY) {
                cs_manage(&robot.angle_cs);
            } else {
                rs_set_angle(&robot.rs, 0); // Sets angle PWM to zero
            }

            if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_DISTANCE_ONLY) {
                cs_manage(&robot.distance_cs);
            } else {
                rs_set_distance(&robot.rs, 0); // Sets distance PWM to zero
            }
        }

        /* Blocking detection manage */
        bd_manage(&robot.angle_bd);
        bd_manage(&robot.distance_bd);

        /* Collision detected */
        if (bd_get(&robot.distance_bd)) {
            WARNING("Collision detected in distance !");
            // trajectory_hardstop(&robot.traj);
            // rs_set_distance(&robot.rs, 0);
        }
        if (bd_get(&robot.angle_bd)) {
            WARNING("Collision detected in angle !");
            // trajectory_hardstop(&robot.traj);
            // rs_set_angle(&robot.rs, 0);
        }

        if (parameter_namespace_contains_changed(control_params)) {
            float kp, ki, kd, ilim;
            pid_get_gains(&robot.angle_pid.pid, &kp, &ki, &kd);
            ilim = pid_get_integral_limit(&robot.angle_pid.pid);
            kp = parameter_scalar_get(parameter_find(control_params, "angle/kp"));
            ki = parameter_scalar_get(parameter_find(control_params, "angle/ki"));
            kd = parameter_scalar_get(parameter_find(control_params, "angle/kd"));
            ilim = parameter_scalar_get(parameter_find(control_params, "angle/ilimit"));
            pid_set_gains(&robot.angle_pid.pid, kp, ki, kd);
            pid_set_integral_limit(&robot.angle_pid.pid, ilim);

            pid_get_gains(&robot.distance_pid.pid, &kp, &ki, &kd);
            ilim = pid_get_integral_limit(&robot.distance_pid.pid);
            kp = parameter_scalar_get(parameter_find(control_params, "distance/kp"));
            ki = parameter_scalar_get(parameter_find(control_params, "distance/ki"));
            kd = parameter_scalar_get(parameter_find(control_params, "distance/kd"));
            ilim = parameter_scalar_get(parameter_find(control_params, "distance/ilimit"));
            pid_set_gains(&robot.distance_pid.pid, kp, ki, kd);
            pid_set_integral_limit(&robot.distance_pid.pid, ilim);
        }

        /* Wait 10 milliseconds (100 Hz) */
        chThdSleepMilliseconds(1000 / ASSERV_FREQUENCY);
    }
}

void base_controller_start(void)
{
    static THD_WORKING_AREA(base_ctrl_thd_wa, BASE_CONTROLLER_STACKSIZE);
    chThdCreateStatic(base_ctrl_thd_wa, sizeof(base_ctrl_thd_wa), BASE_CONTROLLER_PRIO, base_ctrl_thd, NULL);
}

static THD_FUNCTION(position_manager_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while (1) {
        position_manage(&robot.pos);
        chThdSleepMilliseconds(1000 / ODOM_FREQUENCY);
    }
}

void position_manager_start(void)
{
    static THD_WORKING_AREA(position_thd_wa, POSITION_MANAGER_STACKSIZE);
    chThdCreateStatic(position_thd_wa, sizeof(position_thd_wa), POSITION_MANAGER_PRIO, position_manager_thd, NULL);
}

void trajectory_manager_start(void)
{
    static THD_WORKING_AREA(trajectory_thd_wa, TRAJECTORY_MANAGER_STACKSIZE);
    chThdCreateStatic(trajectory_thd_wa, sizeof(trajectory_thd_wa), TRAJECTORY_MANAGER_PRIO, trajectory_manager_thd, &(robot.traj));
}
