#include <thread>
#include <math.h>

#include <error/error.h>

#include <aversive/trajectory_manager/trajectory_manager.h>
#include <aversive/trajectory_manager/trajectory_manager_utils.h>
#include <aversive/trajectory_manager/trajectory_manager_core.h>

#include "main.h"
#include "config.h"

#include "rs_port.h"
#include "base_controller.h"
#include "protobuf/position.pb.h"

#define BASE_CONTROLLER_STACKSIZE 1024
#define POSITION_MANAGER_STACKSIZE 1024
#define TRAJECTORY_MANAGER_STACKSIZE 2048

using namespace std::chrono_literals;

struct _robot robot;

void robot_init(void)
{
    robot.mode = BOARD_MODE_ANGLE_DISTANCE;
    robot.base_speed = BASE_SPEED_SLOW;

    /* Motors */
    static rs_motor_t left_wheel_motor = {.m = &motor_manager, .direction = 1.};
    static rs_motor_t right_wheel_motor = {.m = &motor_manager, .direction = -1.};
    rs_encoder_init();

    robot.angle_pid.divider = 100;
    robot.distance_pid.divider = 100;

    /* Robot system initialisation, encoders and PWM */
    rs_init(&robot.rs);
    rs_set_flags(&robot.rs, RS_USE_EXT);

    rs_set_left_pwm(&robot.rs, rs_left_wheel_set_voltage, &left_wheel_motor);
    rs_set_right_pwm(&robot.rs, rs_right_wheel_set_voltage, &right_wheel_motor);

    rs_set_left_ext_encoder(&robot.rs, rs_encoder_get_left_ext, NULL,
                            config_get_scalar("master/odometry/left_wheel_correction_factor"));
    rs_set_right_ext_encoder(&robot.rs, rs_encoder_get_right_ext, NULL,
                             config_get_scalar("master/odometry/right_wheel_correction_factor"));

    /* Position manager */
    position_init(&robot.pos);
    position_set_related_robot_system(&robot.pos, &robot.rs); // Link pos manager to robot system

    position_set_physical_params(&robot.pos,
                                 config_get_scalar("master/odometry/external_track_mm"),
                                 config_get_scalar("master/odometry/external_encoder_ticks_per_mm"));
    position_use_ext(&robot.pos);

    /* Base angle controller */
    pid_init(&robot.angle_pid.pid);
    quadramp_init(&robot.angle_qr);

    cs_init(&robot.angle_cs);
    cs_set_consign_filter(&robot.angle_cs, quadramp_do_filter, &robot.angle_qr); // Filter acceleration
    cs_set_correct_filter(&robot.angle_cs, cs_pid_process, &robot.angle_pid.pid);
    cs_set_process_in(&robot.angle_cs, rs_set_angle, &robot.rs); // Output on angular virtual pwm
    cs_set_process_out(&robot.angle_cs, rs_get_ext_angle, &robot.rs); // Read angular virtuan encoder
    cs_set_consign(&robot.angle_cs, 0);

    /* Base distance controller */
    pid_init(&robot.distance_pid.pid);
    quadramp_init(&robot.distance_qr);

    cs_init(&robot.distance_cs);
    cs_set_consign_filter(&robot.distance_cs, quadramp_do_filter, &robot.distance_qr); // Filter acceleration
    cs_set_correct_filter(&robot.distance_cs, cs_pid_process, &robot.distance_pid.pid);
    cs_set_process_in(&robot.distance_cs, rs_set_distance, &robot.rs); // Output on distance virtual pwm
    cs_set_process_out(&robot.distance_cs, rs_get_ext_distance, &robot.rs); // Read distance virtuan encoder
    cs_set_consign(&robot.distance_cs, 0);

    /* Trajector manager */
    trajectory_manager_init(&robot.traj, ASSERV_FREQUENCY);
    trajectory_set_cs(&robot.traj, &robot.distance_cs, &robot.angle_cs);
    trajectory_set_robot_params(&robot.traj, &robot.rs, &robot.pos);

    // Distance window, angle window, angle start
    trajectory_set_windows(
        &robot.traj,
        config_get_scalar("master/aversive/trajectories/windows/distance"),
        config_get_scalar("master/aversive/trajectories/windows/angle"),
        config_get_scalar("master/aversive/trajectories/windows/angle_start"));

    /* Initialize blocking detection managers */
    bd_init(&robot.angle_bd);
    bd_init(&robot.distance_bd);

    /* Set calibration side */
    robot.calibration_direction = (enum direction_t)config_get_integer("master/calibration_direction");

    /* Set obstacle inflation sizes */
    robot.robot_size = config_get_integer("master/robot_size_x_mm");
    robot.alignement_length = config_get_integer("master/robot_alignment_length_mm");
    robot.opponent_size = config_get_integer("master/opponent_size_x_mm_default");

    /* Set some defaultj speed and acc. */
    trajectory_set_speed(&robot.traj,
                         speed_mm2imp(&robot.traj, 600.),
                         speed_rd2imp(&robot.traj, 6.));
    trajectory_set_acc(&robot.traj,
                       acc_mm2imp(&robot.traj, 3000.),
                       acc_rd2imp(&robot.traj, 30.));
}

static void base_ctrl_thd(void)
{
    parameter_namespace_t* control_params = parameter_namespace_find(&master_config, "aversive/control");
    parameter_namespace_t* odometry_params = parameter_namespace_find(&master_config, "odometry");

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
        bd_manage(&robot.angle_bd, abs(cs_get_error(&robot.angle_cs)));
        bd_manage(&robot.distance_bd, abs(cs_get_error(&robot.distance_cs)));

        /* Collision detected */
        if (bd_get(&robot.distance_bd)) {
            WARNING("Collision detected in distance !");
        }
        if (bd_get(&robot.angle_bd)) {
            WARNING("Collision detected in angle !");
        }

        if (parameter_namespace_contains_changed(control_params)) {
            float kp, ki, kd, ilim;
            pid_get_gains(&robot.angle_pid.pid, &kp, &ki, &kd);
            kp = parameter_scalar_get(parameter_find(control_params, "angle/kp"));
            ki = parameter_scalar_get(parameter_find(control_params, "angle/ki"));
            kd = parameter_scalar_get(parameter_find(control_params, "angle/kd"));
            ilim = parameter_scalar_get(parameter_find(control_params, "angle/i_limit"));
            pid_set_gains(&robot.angle_pid.pid, kp, ki, kd);
            pid_set_integral_limit(&robot.angle_pid.pid, ilim);

            pid_get_gains(&robot.distance_pid.pid, &kp, &ki, &kd);
            kp = parameter_scalar_get(parameter_find(control_params, "distance/kp"));
            ki = parameter_scalar_get(parameter_find(control_params, "distance/ki"));
            kd = parameter_scalar_get(parameter_find(control_params, "distance/kd"));
            ilim = parameter_scalar_get(parameter_find(control_params, "distance/i_limit"));
            pid_set_gains(&robot.distance_pid.pid, kp, ki, kd);
            pid_set_integral_limit(&robot.distance_pid.pid, ilim);
        }
        if (parameter_namespace_contains_changed(odometry_params)) {
            rs_set_left_ext_encoder(&robot.rs, rs_encoder_get_left_ext, NULL,
                                    config_get_scalar("master/odometry/left_wheel_correction_factor"));
            rs_set_right_ext_encoder(&robot.rs, rs_encoder_get_right_ext, NULL,
                                     config_get_scalar("master/odometry/right_wheel_correction_factor"));

            position_set_physical_params(&robot.pos,
                                         config_get_scalar("master/odometry/external_track_mm"),
                                         config_get_scalar("master/odometry/external_encoder_ticks_per_mm"));
        }

        switch (robot.base_speed) {
            case BASE_SPEED_INIT:
                trajectory_set_speed(&robot.traj,
                                     1000 * speed_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/speed/init")),
                                     speed_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/speed/init")));

                trajectory_set_acc(&robot.traj,
                                   1000 * acc_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/acceleration/init")),
                                   acc_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/acceleration/init")));
                break;

            case BASE_SPEED_SLOW:
                trajectory_set_speed(&robot.traj,
                                     1000 * speed_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/speed/slow")),
                                     speed_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/speed/slow")));

                trajectory_set_acc(&robot.traj,
                                   1000 * acc_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/acceleration/slow")),
                                   acc_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/acceleration/slow")));
                break;

            case BASE_SPEED_FAST:
                trajectory_set_speed(&robot.traj,
                                     1000 * speed_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/speed/fast")),
                                     speed_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/speed/fast")));
                trajectory_set_acc(&robot.traj,
                                   1000 * acc_mm2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/distance/acceleration/fast")),
                                   acc_rd2imp(&robot.traj, config_get_scalar("master/aversive/trajectories/angle/acceleration/fast")));
                break;
            default:
                WARNING("Unknown speed type, going back to safe!");
                robot.base_speed = BASE_SPEED_SLOW;
                break;
        }

        /* Wait until next regulation loop */
        std::this_thread::sleep_for(1000ms / ASSERV_FREQUENCY);
    }
}

void base_controller_start()
{
    std::thread ctrl_thd(base_ctrl_thd);
    ctrl_thd.detach();
}

static void position_manager_thd()
{
    //static TOPIC_DECL(position_topic, RobotPosition);

    //messagebus_advertise_topic(&bus, &position_topic.topic, "/position");

    //RobotPosition pos = RobotPosition_init_zero;

    while (1) {
        position_manage(&robot.pos);
        //pos.x = position_get_x_float(&robot.pos);
        //pos.y = position_get_y_float(&robot.pos);
        //pos.a = position_get_a_rad_float(&robot.pos);
        //messagebus_topic_publish(&position_topic.topic, &pos, sizeof(pos));
        DEBUG_EVERY_N(ODOM_FREQUENCY, "pos: %d %d %d",
                      position_get_x_s16(&robot.pos),
                      position_get_y_s16(&robot.pos),
                      position_get_a_deg_s16(&robot.pos));
        std::this_thread::sleep_for(1s / ODOM_FREQUENCY);
    }
}

void position_manager_start()
{
    std::thread pos_thd(position_manager_thd);
    pos_thd.detach();
}

void trajectory_manager_thd()
{
    while (1) {
        trajectory_manager_manage(&robot.traj);
        std::this_thread::sleep_for(1s / ODOM_FREQUENCY);
    }
}

void trajectory_manager_start()
{
    std::thread traj_thd(trajectory_manager_thd);
    traj_thd.detach();
}
