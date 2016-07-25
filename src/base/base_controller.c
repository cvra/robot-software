#include <ch.h>
#include <math.h>
#include "polar.h"
#include "base_controller.h"
#include "main.h"
#include "cvra/cvra_motors.h"
#include "trajectory_manager/trajectory_manager_core.h"


#define BASE_CONTROLLER_STACKSIZE    2048
#define POSITION_MANAGER_STACKSIZE   1024
#define TRAJECTORY_MANAGER_STACKSIZE 1024


struct _robot robot;


void base_controller_init(base_controller_t *base)
{
    pid_init(&(base->distance_pid));
    pid_init(&(base->heading_pid));
}

void base_controller_compute_error(polar_t *error, pose2d_t desired, pose2d_t measured)
{
    error->distance = sqrtf(powf(desired.x - measured.x, 2)
                            + powf(desired.y - measured.y, 2));

    error->angle = angle_delta(measured.heading, desired.heading);
}


static THD_FUNCTION(base_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    /* Motors */
    cvra_motor_t left_wheel_motor = {.m=&motor_manager, .max_velocity=10.f};
    cvra_motor_t right_wheel_motor = {.m=&motor_manager, .max_velocity=10.f};

    /* Robot system initialisation, encoders and PWM */
    rs_init(&robot.rs);
    rs_set_flags(&robot.rs, RS_USE_EXT);

    rs_set_left_pwm(&robot.rs, cvra_motor_left_wheel_set_velocity, &left_wheel_motor);
    rs_set_right_pwm(&robot.rs, cvra_motor_right_wheel_set_velocity, &right_wheel_motor);
    rs_set_left_ext_encoder(&robot.rs, cvra_encoder_get_left_ext, NULL, 1.);
    rs_set_right_ext_encoder(&robot.rs, cvra_encoder_get_right_ext, NULL, -1.);

    /* Position manager */
    position_init(&robot.pos);
    position_set_related_robot_system(&robot.pos, &robot.rs); // Link pos manager to robot system

    position_set_physical_params(&robot.pos, 193.82313537598, 162.9746617261);
    position_use_ext(&robot.pos);

    /* Base angle controller */
    pid_init(&robot.angle_pid.pid);
    pid_set_gains(&robot.angle_pid.pid, 400, 0, 2000);
    pid_set_integral_limit(&robot.angle_pid.pid, 5000);

    quadramp_init(&robot.angle_qr);

    cs_init(&robot.angle_cs);
    cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr); // Filter acceleration
    cs_set_correct_filter(&robot.angle_cs, cvra_pid_process, &robot.angle_pid.pid);
    cs_set_process_in(&robot.angle_cs, rs_set_angle, &robot.rs); // Output on angular virtual pwm
    cs_set_process_out(&robot.angle_cs, rs_get_ext_angle, &robot.rs); // Read angular virtuan encoder
    cs_set_consign(&robot.angle_cs, 0);

    /* Base distance controller */
    pid_init(&robot.distance_pid.pid);
    pid_set_gains(&robot.distance_pid.pid, 200, 0, 1000);
    pid_set_integral_limit(&robot.distance_pid.pid, 5000);

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

    /* distance window, angle window, angle start */
    trajectory_set_windows(&robot.traj, 15., 5.0, 10.); // Prod

    // Angle BDM
    bd_init(&robot.angle_bd, &robot.angle_cs);
    bd_set_thresholds(&robot.angle_bd, 3000, 1); /* thresold, duration. */

    // Distance BDM
    bd_init(&robot.distance_bd, &robot.distance_cs);
    bd_set_thresholds(&robot.distance_bd, 3600, 1); /* thresold, duration. */

    robot.is_aligning = 0;

    // Position initialisation
    position_set(&robot.pos, 0, 0, 0);

    while (1) {
        rs_update(&robot.rs);

        /* Gestion de l'asservissement. */
        if (robot.mode != BOARD_MODE_SET_PWM) {
            if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_ANGLE_ONLY) {
                cs_manage(&robot.angle_cs);
            } else {
                rs_set_angle(&robot.rs, 0); // Sets a null angle PWM
            }

            if (robot.mode == BOARD_MODE_ANGLE_DISTANCE || robot.mode == BOARD_MODE_DISTANCE_ONLY) {
                cs_manage(&robot.distance_cs);
            } else {
                rs_set_distance(&robot.rs, 0); // Sets a distance angle PWM
            }
        }

        /* Gestion du blocage */
        bd_manage(&robot.angle_bd);
        bd_manage(&robot.distance_bd);

        /* Wait 10 milliseconds (100 Hz) */
        chThdSleepMilliseconds(1000 / ASSERV_FREQUENCY);
    }
}

void base_controller_start(void)
{
    static THD_WORKING_AREA(base_ctrl_thd_wa, BASE_CONTROLLER_STACKSIZE);
    chThdCreateStatic(base_ctrl_thd_wa, sizeof(base_ctrl_thd_wa), NORMALPRIO, base_ctrl_thd, NULL);
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
    chThdCreateStatic(position_thd_wa, sizeof(position_thd_wa), NORMALPRIO, position_manager_thd, NULL);
}

void trajectory_manager_start(void)
{
    static THD_WORKING_AREA(trajectory_thd_wa, TRAJECTORY_MANAGER_STACKSIZE);
    chThdCreateStatic(trajectory_thd_wa, sizeof(trajectory_thd_wa), NORMALPRIO, trajectory_manager_thd, &(robot.traj));
}
