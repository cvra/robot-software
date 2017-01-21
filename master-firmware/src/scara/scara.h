#ifndef SCARA_H
#define SCARA_H

#include <aversive/position_manager/position_manager.h>
#include <aversive/math/vect2/vect2.h>

#include "scara_kinematics.h"
#include "scara_waypoint.h"

#include <aversive_port/cvra_pid.h>
#include <aversive/control_system_manager/control_system_manager.h>

typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    /* Physical parameters. */
    float length[2];                  /**< Length of the 2 arms elements. */

    /* Path informations */
    scara_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    semaphore_t trajectory_semaphore;
    int32_t last_loop;              /**< Timestamp of the last loop execution, in us since boot. */
    struct robot_position *robot_pos;

    shoulder_mode_t shoulder_mode;
} scara_t;


void scara_init(scara_t *arm);

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj);

void scara_set_physical_parameters(scara_t *arm);

void scara_manage(scara_t *arm);


scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date);

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos);

void scara_shutdown(scara_t *arm);

#endif
