#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cvra/cvra_pid.h"
#include "base.h"
#include "robot_system/robot_system.h"
#include "position_manager/position_manager.h"
#include "quadramp/quadramp.h"
#include "control_system_manager/control_system_manager.h"
#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"


/** Frequency of the regulation loop and odometry loop (in Hz) */
#define ASSERV_FREQUENCY 100
#define ODOM_FREQUENCY   50

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
    BOARD_MODE_ANGLE_ONLY,     ///< Angle regulated only
    BOARD_MODE_DISTANCE_ONLY,  ///< Distance regulated only
    BOARD_MODE_INDEPENDENT,    ///< 3 independent axis
    BOARD_MODE_FREE,           ///< No control system
    BOARD_MODE_SET_PWM,
};

/**
 @brief contains all global vars.

 This structure contains all vars that should be global. This is a clean way to
 group all vars in one place. It also serve as a namespace.
 */
struct _robot {
    struct robot_system rs;    // Robot system (angle & distance)
    struct robot_position pos; // Position manager

    struct cs angle_cs;        // Control system manager for angle
    struct cs distance_cs;     // Control system manager for distance
    cvra_pid_t angle_pid;
    cvra_pid_t distance_pid;
    struct quadramp_filter angle_qr;
    struct quadramp_filter distance_qr;

    struct trajectory traj;                 // Trivial trajectory manager
    struct blocking_detection angle_bd;     // Angle blocking detection manager
    struct blocking_detection distance_bd;  // Distance blocking detection manager

    enum board_mode_t mode;                 // The current board mode

    uint8_t is_aligning:1;                  // =1 if the robot is aligning on border
};

typedef struct {
    pid_ctrl_t distance_pid;
    pid_ctrl_t heading_pid;
} base_controller_t;

void base_controller_init(base_controller_t *base);

void base_controller_compute_error(polar_t *error, pose2d_t desired, pose2d_t measured);

void base_controller_start(void);

void position_manager_start(void);


#ifdef __cplusplus
}
#endif

#endif /* BASE_CONTROLLER_H */
