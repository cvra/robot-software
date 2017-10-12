#ifndef SCARA_H
#define SCARA_H

#include <ch.h>

#include <aversive/position_manager/position_manager.h>
#include <aversive/math/vect2/vect2.h>
#include <error/error.h>

#include <pid/pid.h>
#include "scara_kinematics.h"
#include "scara_waypoint.h"

#include <aversive_port/cvra_pid.h>
#include <aversive/control_system_manager/control_system_manager.h>

/** Control mode of the scara arm. */
typedef enum {
    CONTROL_JOINT_POSITION=0, /**< Control the motors in position directly, ugly performance, but safe. */
    CONTROL_JAM_PID_XYA,      /**< Control using jacobian and PIDs on x,y,a, smooth cartesian trajectories. */
} scara_control_mode_t;

typedef struct {
    vect2_cart offset_xy; /**< Offset vector between center of robot and shoulder. */
    float offset_rotation; /**< Rotation between the robot base and shoulder in rad. */

    /* Motor control callbacks */
    void (*set_z_position)(void*, float);        /**< Callback function to set z position. */
    void (*set_shoulder_position)(void*, float); /**< Callback function to set shoulder position. */
    void (*set_elbow_position)(void*, float);    /**< Callback function to set elbow position. */
    void (*set_wrist_position)(void*, float, float);    /**< Callback function to set wrist position. */

    void (*set_shoulder_velocity)(void*, float); /**< Callback function to set shoulder velocity. */
    void (*set_elbow_velocity)(void*, float);    /**< Callback function to set elbow velocity. */
    void (*set_wrist_velocity)(void*, float, float);    /**< Callback function to set wrist velocity. */

    /* Motor feedback callbacks */
    float (*get_z_position)(void*);        /**< Callback function to get z position. */
    float (*get_shoulder_position)(void*); /**< Callback function to get shoulder position. */
    float (*get_elbow_position)(void*);    /**< Callback function to get elbow position. */
    void (*get_wrist_position)(void*, float*, float*); /**< Callback function to get wrist position. */

    /* Motor control args */
    void* z_args;
    void* shoulder_args;
    void* elbow_args;
    void* wrist_args;

    /* Motor positions */
    float z_pos;
    float shoulder_pos;
    float elbow_pos;

    /* Control system */
    pid_ctrl_t x_pid;
    pid_ctrl_t y_pid;

    /* Physical parameters. */
    float length[2];                  /**< Length of the 2 arms elements. */

    /* Path informations */
    scara_trajectory_t trajectory;    /**< Current trajectory of the arm. */
    semaphore_t trajectory_semaphore;
    int32_t last_loop;              /**< Timestamp of the last loop execution, in us since boot. */
    struct robot_position *robot_pos;
    int kinematics_solution_count;

    shoulder_mode_t shoulder_mode;
    scara_control_mode_t control_mode;

    mutex_t lock;
} scara_t;


void scara_init(scara_t *arm);

void scara_set_physical_parameters(scara_t* arm, float upperarm_length, float forearm_length);
void scara_set_offset(scara_t* arm, float offset_x, float offset_y, float offset_rotation);

void scara_set_z_callbacks(scara_t* arm, void (*set_z_position)(void*, float),
                           float (*get_z_position)(void*), void* z_args);
void scara_set_shoulder_callbacks(scara_t* arm, void (*set_shoulder_position)(void*, float),
                                  void (*set_shoulder_velocity)(void*, float),
                                  float (*get_shoulder_position)(void*), void* shoulder_args);
void scara_set_elbow_callbacks(scara_t* arm, void (*set_elbow_position)(void*, float),
                               void (*set_elbow_velocity)(void*, float),
                               float (*get_elbow_position)(void*), void* elbow_args);
void scara_set_wrist_callbacks(scara_t* arm, void (*set_wrist_position)(void*, float, float),
                               void (*set_wrist_velocity)(void*, float, float),
                               void (*get_wrist_position)(void*, float*, float*), void* wrist_args);


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
void scara_goto_with_length(scara_t* arm, float x, float y, float z, scara_coordinate_t system,
                            const float duration);

/* Move arm in axis only */
void scara_move_z(scara_t* arm, float z, scara_coordinate_t system, const float duration);

/* Get current arm position */
void scara_pos(scara_t* arm, float* x, float* y, float* z, scara_coordinate_t system);

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj);

void scara_manage(scara_t *arm);


scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date);

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos);

void scara_shutdown(scara_t *arm);

#endif
