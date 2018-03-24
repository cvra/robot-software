#include <string.h>
#include <math.h>

#include "scara.h"
#include "scara_kinematics.h"
#include "scara_trajectories.h"
#include "scara_utils.h"
#include "scara_port.h"

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
    arm->control_mode = CONTROL_JOINT;

    arm->time_offset = 0;

    /* Configure arm controllers */
    scara_joint_controller_init(&arm->joint_controller);
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

void scara_control_mode_joint(scara_t* arm)
{
    arm->control_mode = CONTROL_JOINT;
}

void scara_control_mode_cartesian(scara_t* arm)
{
    arm->control_mode = CONTROL_CARTESIAN;
}

void scara_control_mode_disabled(scara_t* arm)
{
    arm->control_mode = CONTROL_DISABLED;
}


void scara_hold_position(scara_t* arm, scara_coordinate_t system)
{
    position_3d_t current_pos = scara_position(arm, system);
    velocity_3d_t max_vel = {.x=300, .y=300, .z=1000};
    scara_goto(arm, current_pos, system, max_vel);
}

void scara_goto(scara_t* arm, position_3d_t pos, scara_coordinate_t system, velocity_3d_t max_vel)
{
    scara_trajectory_init(&(arm->trajectory));
    if (arm->control_mode == CONTROL_CARTESIAN) {
        scara_trajectory_append_point(&(arm->trajectory), scara_position(arm, system), system, max_vel, arm->length);
    }
    scara_trajectory_append_point(&(arm->trajectory), pos, system, max_vel, arm->length);
    scara_do_trajectory(arm, &(arm->trajectory));
}

void scara_move_z(scara_t* arm, float z_new, scara_coordinate_t system, float max_vel_z)
{
    position_3d_t pos = scara_position(arm, system);
    pos.z = z_new;
    velocity_3d_t max_vel = {.x=0, .y=0, .z=max_vel_z};
    scara_goto(arm, pos, system, max_vel);
}

position_3d_t scara_position(scara_t* arm, scara_coordinate_t system)
{
    point_t pos = scara_forward_kinematics(
        arm->joint_positions.shoulder, arm->joint_positions.elbow, arm->length);

    if (system == COORDINATE_ROBOT) {
        pos = scara_coordinate_arm2robot(pos, arm->offset_xy, arm->offset_rotation);
    } else if (system == COORDINATE_TABLE) {
        pos = scara_coordinate_arm2robot(pos, arm->offset_xy, arm->offset_rotation);

        point_t robot_xy = {.x = position_get_x_float(arm->robot_pos), .y = position_get_y_float(
                                arm->robot_pos)};
        float robot_a = position_get_a_rad_float(arm->robot_pos);

        pos = scara_coordinate_robot2table(pos, robot_xy, robot_a);
    }

    position_3d_t position = {.x = pos.x, .y = pos.y, .z = arm->joint_positions.z};
    return position;
}

void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj)
{
    scara_lock(&arm->lock);
    scara_trajectory_copy(&arm->trajectory, traj);
    scara_unlock(&arm->lock);
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
    if (arm->control_mode == CONTROL_DISABLED) {
        scara_hw_shutdown_joints(&arm->hw_interface);
        scara_unlock(&arm->lock);
        return;
    }

    scara_waypoint_t frame = scara_position_for_date(arm, current_date);

    shoulder_mode_t mode = scara_orientation_mode(arm->shoulder_mode, arm->offset_rotation);
    scara_joint_setpoints_t joint_setpoints;

    if (arm->control_mode == CONTROL_CARTESIAN) {
        scara_ik_controller_set_geometry(&arm->ik_controller, frame.length);
        joint_setpoints = scara_ik_controller_process(
            &arm->ik_controller, frame.position, arm->joint_positions);

        position_3d_t measured = scara_position(arm, frame.coordinate_type);
        DEBUG("Arm x %.3f y %.3f Arm velocities %.3f %.3f",
              measured.x, measured.y, joint_setpoints.shoulder.value, joint_setpoints.elbow.value);
    } else if (arm->control_mode == CONTROL_JOINT) {
        scara_joint_controller_set_geometry(&arm->joint_controller, frame.length, mode);
        joint_setpoints = scara_joint_controller_process(
            &arm->joint_controller, frame.position, arm->joint_positions);
    }
    scara_hw_set_joints(&arm->hw_interface, joint_setpoints);

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
            &arm->trajectory, current_pos.position, current_pos.coordinate_type,
            current_pos.max_velocity, &current_pos.length[0]);
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
