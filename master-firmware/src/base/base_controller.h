#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <quadramp/quadramp.h>

#include <aversive/blocking_detection_manager/blocking_detection_manager.h>
#include <aversive/control_system_manager/control_system_manager.h>
#include <aversive/position_manager/position_manager.h>
#include <aversive/robot_system/robot_system.h>
#include <aversive/trajectory_manager/trajectory_manager.h>

#include "cs_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Frequency of the regulation loop and odometry loop (in Hz) */
#define ASSERV_FREQUENCY 100
#define ODOM_FREQUENCY 100
#define TRAJECTORY_EVENT_FREQUENCY 100

/**
 @brief Type of the regulators

 This enum holds the type of regulators : Angle and distance, distance only,
 angle only, etc...
 @note Some values are unused :
 - BOARD_MODE_INDEPENDENT
 - BOARD_MODE_FREE
 - BOARD_MODE_OTHER
 */
enum board_mode_t {
    BOARD_MODE_ANGLE_DISTANCE, ///< Angle & Distance regulated
    BOARD_MODE_ANGLE_ONLY, ///< Angle regulated only
    BOARD_MODE_DISTANCE_ONLY, ///< Distance regulated only
    BOARD_MODE_INDEPENDENT, ///< 3 independent axis
    BOARD_MODE_FREE, ///< No control system
    BOARD_MODE_SET_PWM,
};

enum direction_t {
    DIRECTION_BACKWARD = -1,
    DIRECTION_FORWARD = 1,
};

enum base_speed_t {
    BASE_SPEED_INIT,
    BASE_SPEED_SLOW,
    BASE_SPEED_FAST
};

/**
 @brief contains all global vars.

 This structure contains all vars that should be global. This is a clean way to
 group all vars in one place. It also serve as a namespace.
 */
struct _robot {
    struct robot_system rs; // Robot system (angle & distance)
    struct robot_position pos; // Position manager
    enum base_speed_t base_speed;

    struct cs angle_cs; // Control system manager for angle
    struct cs distance_cs; // Control system manager for distance
    cs_pid_t angle_pid;
    cs_pid_t distance_pid;
    struct quadramp_filter angle_qr;
    struct quadramp_filter distance_qr;

    struct trajectory traj; // Trivial trajectory manager
    struct blocking_detection angle_bd; // Angle blocking detection manager
    struct blocking_detection distance_bd; // Distance blocking detection manager

    enum board_mode_t mode; // The current board mode

    enum direction_t calibration_direction; // Calibration direction / side of the robot
    int robot_size;
    int alignement_length;
    int opponent_size;

    uint32_t start_time; // Time since the beginning of the match, in microseconds
};

extern struct _robot robot;

void robot_init(void);
void robot_trajectory_windows_set_coarse(void);
void robot_trajectory_windows_set_fine(void);

void base_controller_start(void);

void position_manager_start(void);

void trajectory_manager_start(void);

#ifdef __cplusplus
}
#endif

#endif /* BASE_CONTROLLER_H */
