#ifndef SCARA_H
#define SCARA_H

#include <ch.h>

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
    void (*set_z_position)(void*, float);        /**< Callback function to set z position. */
    void (*set_shoulder_position)(void*, float); /**< Callback function to set shoulder position. */
    void (*set_elbow_position)(void*, float);    /**< Callback function to set elbow position. */
    void (*set_wrist_position)(void*, float);    /**< Callback function to set wrist position. */

    /* Motor feedback callbacks */
    float (*get_z_position)(void*);        /**< Callback function to get z position. */
    float (*get_shoulder_position)(void*); /**< Callback function to get shoulder position. */
    float (*get_elbow_position)(void*);    /**< Callback function to get elbow position. */
    float (*get_wrist_position)(void*);        /**< Callback function to get wrist position. */

    /* Motor control args */
    void* z_args;
    void* shoulder_args;
    void* elbow_args;
    void* wrist_args;

    /* Motor positions */
    float z_pos;
    float shoulder_pos;
    float elbow_pos;
    float wrist_pos;

    /* Physical parameters. */
    float length[3];                  /**< Length of the 2 arms elements and wrist to hand center. */

    /* Path informations */
    scara_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    semaphore_t trajectory_semaphore;
    int32_t last_loop;              /**< Timestamp of the last loop execution, in us since boot. */
    struct robot_position *robot_pos;

    shoulder_mode_t shoulder_mode;

    mutex_t lock;
} scara_t;


void scara_init(scara_t *arm);

void scara_set_physical_parameters(scara_t* arm, float upperarm_length, float forearm_length, float hand_length);
void scara_set_offset(scara_t* arm, float offset_x, float offset_y, float offset_rotation);

void scara_set_z_callbacks(scara_t* arm, void (*set_z_position)(void*, float),
                           float (*get_z_position)(void*), void* z_args);
void scara_set_shoulder_callbacks(scara_t* arm, void (*set_shoulder_position)(void*, float),
                                  float (*get_shoulder_position)(void*), void* shoulder_args);
void scara_set_elbow_callbacks(scara_t* arm, void (*set_elbow_position)(void*, float),
                               float (*get_elbow_position)(void*), void* elbow_args);
void scara_set_wrist_callbacks(scara_t* arm, void (*set_wrist_position)(void*, float),
                               float (*get_wrist_position)(void*), void* wrist_args);

/* Goto position in specified coordinate system */
void scara_goto(scara_t* arm, float x, float y, float z, scara_coordinate_t system, const float duration);

/* Get current arm position */
void scara_pos(scara_t* arm, float* x, float* y, float* z, float* a, scara_coordinate_t system);

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj);

void scara_manage(scara_t *arm);


scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date);

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos);

void scara_shutdown(scara_t *arm);

#endif
