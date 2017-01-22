#ifndef SCARA_H
#define SCARA_H

#include <aversive/position_manager/position_manager.h>
#include <aversive/math/vect2/vect2.h>
#include <error/error.h>

#include "scara_kinematics.h"
#include "scara_waypoint.h"

#include <aversive_port/cvra_pid.h>
#include <aversive/control_system_manager/control_system_manager.h>


typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    /* Motor control callbacks */
    void (*set_shoulder_position)(float); /**< Callback function to set shoulder position. */
    void (*set_elbow_position)(float);    /**< Callback function to set elbow position. */

    /* Motor directions */
    int shoulder_dir;
    int elbow_dir;

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

void scara_set_physical_parameters(scara_t* arm, float upperarm_length, float forearm_length);
void scara_set_offset(scara_t* arm, float offset_x, float offset_y, float offset_rotation);
void scara_set_motor_direction(scara_t* arm, int shoulder_dir, int elbow_dir);

void scara_set_shoulder_callback(scara_t* arm, void (*set_shoulder_position)(float));
void scara_set_elbow_callback(scara_t* arm, void (*set_elbow_position)(float));

/* Goto position in arm local coordinate system */
void scara_goto_arm(scara_t* arm, float x, float y);

/* Goto position in robot coordinate system */
void scara_goto_robot(scara_t* arm, float x, float y);

/* Goto position in table coordinate system */
void scara_goto_table(scara_t* arm, float x, float y, float robot_x, float robot_y, float robot_a);

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj);

void scara_manage(scara_t *arm);


scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date);

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos);

void scara_shutdown(scara_t *arm);

#endif
