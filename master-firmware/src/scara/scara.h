#ifndef SCARA_H
#define SCARA_H

#include <ch.h>

#include <aversive/control_system_manager/control_system_manager.h>
#include <aversive/math/vect2/vect2.h>
#include <aversive/position_manager/position_manager.h>
#include <aversive_port/cvra_pid.h>
#include <error/error.h>

#include <pid/pid.h>

#include "scara_hardware_interface.h"
#include "scara_kinematics.h"
#include "scara_waypoint.h"
#include "control/scara_inverse_kinematics_controller.h"

/** Control mode of the scara arm. */
typedef enum {
    CONTROL_JOINT_POSITION=0, /**< Control the motors in position directly, ugly performance, but safe. */
    CONTROL_JAM_PID_XYA,      /**< Control using jacobian and PIDs on x,y,a, smooth cartesian trajectories. */
} scara_control_mode_t;


/** Scara arm datastruct */
typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    scara_hardware_interface_t hw_interface; /**< Hardware interface handling joint IO */
    scara_joint_positions_t joint_positions; /**< Cached joint positions */

    /* Control system */
    scara_ik_controller_t ik_controller;

    /* Physical parameters. */
    float length[2];                  /**< Length of the 2 arms elements. */

    /* Path informations */
    scara_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    struct robot_position *robot_pos;

    shoulder_mode_t shoulder_mode;
    scara_control_mode_t control_mode;

    /* Cached state for pause/continue */
    int32_t time_offset; /**< in us */
    scara_trajectory_t previous_trajectory; /**< Paused trajectory */

    mutex_t lock;
} scara_t;


void scara_init(scara_t *arm);

void scara_set_physical_parameters(scara_t* arm, float upperarm_length, float forearm_length);
void scara_set_offset(scara_t* arm, float offset_x, float offset_y, float offset_rotation);

/** Enable "ugly mode" which is the joint position control
 * where each motor is controlled in position
 */
void scara_ugly_mode_enable(scara_t* arm);

/** Disables "ugly mode" and enables JAM PID controller
 * where each motor is controlled in velocity (except z axis)
 */
void scara_ugly_mode_disable(scara_t* arm);


/* Goto position in specified coordinate system */
void scara_goto(scara_t* arm, float x, float y, float z, scara_coordinate_t system, const float duration);

/* Move arm in axis only */
void scara_move_z(scara_t* arm, float z, scara_coordinate_t system, const float duration);

/* Get current arm position */
position_3d_t scara_position(scara_t* arm, scara_coordinate_t system);

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj);

void scara_manage(scara_t *arm);

bool scara_compute_joint_angles(scara_t* arm, scara_waypoint_t frame, float* alpha, float* beta);

scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date);

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos);

void scara_shutdown(scara_t *arm);

void scara_pause(scara_t* arm);
void scara_continue(scara_t* arm);

#endif
