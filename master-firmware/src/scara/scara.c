#include <string.h>
#include <math.h>

#include "scara.h"
#include "scara_kinematics.h"
#include "scara_trajectories.h"
#include "scara_utils.h"
#include "scara_port.h"
#include "control/scara_joint_controller.h"

static void scara_lock(mutex_t* mutex)
{
    chMtxLock(mutex);
}

static void scara_unlock(mutex_t* mutex)
{
    chMtxUnlock(mutex);
}

void scara_init(scara_t *arm)
{
    memset(arm, 0, sizeof(scara_t));

    arm->shoulder_mode = SHOULDER_BACK;
    arm->control_mode = CONTROL_JOINT_POSITION;

    arm->time_offset = 0;

    /* Configure arm controllers */
    scara_ik_controller_init(&arm->ik_controller);

    chMtxObjectInit(&arm->lock);
}

void scara_set_physical_parameters(scara_t *arm, float upperarm_length, float forearm_length)
{
    arm->length[0] = upperarm_length;
    arm->length[1] = forearm_length;
}

void scara_set_offset(scara_t* arm, float offset_x, float offset_y, float offset_rotation)
{
    arm->offset_xy.x = offset_x;
    arm->offset_xy.y = offset_y;
    arm->offset_rotation = offset_rotation;
}

void scara_ugly_mode_enable(scara_t* arm)
{
    arm->control_mode = CONTROL_JOINT_POSITION;
}

void scara_ugly_mode_disable(scara_t* arm)
{
    arm->control_mode = CONTROL_JAM_PID_XYA;
}


void scara_goto(scara_t* arm, float x, float y, float z, scara_coordinate_t system, const float duration)
{
    scara_trajectory_init(&(arm->trajectory));
    scara_trajectory_append_point(&(arm->trajectory), x, y, z, system, duration, arm->length);
    scara_do_trajectory(arm, &(arm->trajectory));
}

void scara_move_z(scara_t* arm, float z_new, scara_coordinate_t system, const float duration)
{
    float x, y, z;
    scara_pos(arm, &x, &y, &z, system);
    scara_goto(arm, x, y, z_new, system, duration);
}

void scara_pos(scara_t* arm, float* x, float* y, float* z, scara_coordinate_t system)
{
    point_t pos;
    pos = scara_forward_kinematics(arm->joint_positions.shoulder, arm->joint_positions.elbow, arm->length);

    if (system == COORDINATE_ROBOT) {
        pos = scara_coordinate_arm2robot(pos, arm->offset_xy, arm->offset_rotation);
    } else if (system == COORDINATE_TABLE) {
        pos = scara_coordinate_arm2robot(pos, arm->offset_xy, arm->offset_rotation);

        point_t robot_xy = {.x = position_get_x_float(arm->robot_pos), .y = position_get_y_float(
                                arm->robot_pos)};
        float robot_a = position_get_a_rad_float(arm->robot_pos);

        pos = scara_coordinate_robot2table(pos, robot_xy, robot_a);
    }

    *x = pos.x;
    *y = pos.y;
    *z = arm->joint_positions.z;
}

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj)
{
    scara_lock(&arm->lock);
    scara_trajectory_copy(&arm->trajectory, traj);
    scara_unlock(&arm->lock);
}

bool scara_compute_joint_angles(scara_t* arm, scara_waypoint_t frame, float* alpha, float* beta)
{
    point_t target = {.x = frame.position.x, .y = frame.position.y};

    point_t p1, p2;
    int kinematics_solution_count = scara_num_possible_elbow_positions(target, arm->length[0], arm->length[1], &p1, &p2);

    if (kinematics_solution_count == 0) {
        return false;
    } else if (kinematics_solution_count == 2) {
        shoulder_mode_t mode = scara_orientation_mode(arm->shoulder_mode, arm->offset_rotation);
        p1 = scara_shoulder_solve(target, p1, p2, mode);
    }

    /* p1 now contains the correct elbow pos. */
    *alpha = scara_compute_shoulder_angle(p1, target);
    *beta = scara_compute_elbow_angle(p1, target);

    /* This is due to mecanical construction of the arms. */
    *beta = *beta - *alpha;

    /* The arm cannot make one full turn. */
    if (*beta < -M_PI) {
        *beta = 2 * M_PI + *beta;
    }
    if (*beta > M_PI) {
        *beta = *beta - 2 * M_PI;
    }

    return true;
}

void scara_manage(scara_t *arm)
{
    int32_t current_date = scara_time_get() + arm->time_offset;

    scara_lock(&arm->lock);

    arm->joint_positions = scara_hw_read_joint_positions(&arm->hw_interface);

    if (scara_trajectory_is_empty(&arm->trajectory)) {
        scara_unlock(&arm->lock);
        return;
    }

    scara_waypoint_t frame = scara_position_for_date(arm, current_date);

    float alpha, beta;
    bool solution_found = scara_compute_joint_angles(arm, frame, &alpha, &beta);

    if (solution_found) {
        DEBUG("Inverse kinematics: Found a solution");
    } else {
        DEBUG("Inverse kinematics: Found no solution, disabling the arm");
        scara_hw_shutdown_joints(&arm->hw_interface);
        scara_unlock(&arm->lock);
        return;
    }

    if (arm->control_mode == CONTROL_JAM_PID_XYA) {
        position_3d_t measured;
        scara_pos(arm, &measured.x, &measured.y, &measured.z, COORDINATE_ARM);

        scara_ik_controller_set_geometry(&arm->ik_controller, frame.length);

        scara_joint_setpoints_t joint_setpoints =
            scara_ik_controller_process(&arm->ik_controller, measured, frame.position,
                                        arm->joint_positions);

        scara_hw_set_joints(&arm->hw_interface, joint_setpoints);

        scara_pos(arm, &measured.x, &measured.y, &measured.z, frame.coordinate_type);
        DEBUG("Arm x %.3f y %.3f Arm velocities %.3f %.3f",
              measured.x, measured.y, joint_setpoints.shoulder.value, joint_setpoints.elbow.value);
    } else {
        scara_joint_positions_t joints_desired = {
            .z = frame.position.z, .shoulder = alpha, .elbow = beta
        };
        scara_joint_setpoints_t joint_setpoints =
            scara_joint_controller_process(joints_desired, arm->joint_positions);
        scara_hw_set_joints(&arm->hw_interface, joint_setpoints);
    }

    scara_unlock(&arm->lock);
}

static scara_waypoint_t scara_convert_waypoint_coordinate(scara_t *arm, scara_waypoint_t key)
{
    point_t pos = {.x = key.position.x, .y = key.position.y};

    if (key.coordinate_type == COORDINATE_TABLE) {
        point_t robot_pos;
        float robot_a_rad;

        robot_pos.x = position_get_x_float(arm->robot_pos);
        robot_pos.y = position_get_y_float(arm->robot_pos);
        robot_a_rad = position_get_a_rad_float(arm->robot_pos);
        pos = scara_coordinate_table2robot(pos, robot_pos, robot_a_rad);
        pos = scara_coordinate_robot2arm(pos, arm->offset_xy, arm->offset_rotation);
    } else if (key.coordinate_type == COORDINATE_ROBOT) {
        pos = scara_coordinate_robot2arm(pos, arm->offset_xy, arm->offset_rotation);
    }

    key.position.x = pos.x;
    key.position.y = pos.y;
    return key;
}

scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date)
{
    int i = 0;
    scara_waypoint_t k1, k2;

    /* If we are past last waypoint, simply return last frame. */
    if (arm->trajectory.frames[arm->trajectory.frame_count - 1].date < date) {
        k1 = arm->trajectory.frames[arm->trajectory.frame_count - 1];
        return scara_convert_waypoint_coordinate(arm, k1);
    }

    while (arm->trajectory.frames[i].date < date) {
        i++;
    }

    k1 = scara_convert_waypoint_coordinate(arm, arm->trajectory.frames[i - 1]);
    k2 = scara_convert_waypoint_coordinate(arm, arm->trajectory.frames[i]);

    return scara_trajectory_interpolate_waypoints(k1, k2, date);
}

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos)
{
    arm->robot_pos = pos;
}

void scara_shutdown(scara_t *arm)
{
    scara_trajectory_delete(&arm->trajectory);
}

static bool scara_is_paused(scara_t* arm)
{
    return !scara_trajectory_is_empty(&arm->previous_trajectory);
}

void scara_pause(scara_t* arm)
{
    scara_lock(&arm->lock);

    if (!scara_is_paused(arm)) {
        arm->time_offset = scara_time_get();

        scara_trajectory_copy(&arm->previous_trajectory, &arm->trajectory);

        scara_waypoint_t current_pos = scara_position_for_date(arm, arm->time_offset);
        scara_trajectory_delete(&arm->trajectory);
        scara_trajectory_append_point(
            &arm->trajectory, current_pos.position.x, current_pos.position.y,
            current_pos.position.z, current_pos.coordinate_type,
            current_pos.date / 1e6, &current_pos.length[0]);
    }

    scara_unlock(&arm->lock);
}

void scara_continue(scara_t* arm)
{
    scara_lock(&arm->lock);

    if (scara_is_paused(arm)) {
        arm->time_offset = arm->time_offset - scara_time_get();

        scara_trajectory_copy(&arm->trajectory, &arm->previous_trajectory);
        scara_trajectory_delete(&arm->previous_trajectory);
    }

    scara_unlock(&arm->lock);
}
