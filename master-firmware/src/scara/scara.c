#include <string.h>
#include <math.h>

#include "scara.h"
#include "scara_kinematics.h"
#include "scara_trajectories.h"
#include "scara_utils.h"
#include "scara_port.h"

void scara_init(scara_t *arm)
{
    memset(arm, 0, sizeof(scara_t));

    /* Sets last loop run date for lag compensation. */
    arm->last_loop = scara_time_get();

    arm->shoulder_mode = SHOULDER_BACK;
}


void scara_set_physical_parameters(scara_t* arm, float upperarm_length, float forearm_length)
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

void scara_set_motor_direction(scara_t* arm, int shoulder_dir, int elbow_dir)
{
    arm->shoulder_dir = shoulder_dir;
    arm->elbow_dir = elbow_dir;
}


void scara_set_shoulder_callback(scara_t* arm, void (*set_shoulder_position)(float))
{
    arm->set_shoulder_position = set_shoulder_position;
}

void scara_set_elbow_callback(scara_t* arm, void (*set_elbow_position)(float))
{
    arm->set_elbow_position = set_elbow_position;
}

void scara_goto_arm(scara_t* arm, float x, float y)
{
    point_t target = {.x = x, .y = y};

    point_t p1, p2;
    int position_count = scara_num_possible_elbow_positions(target, arm->length[0], arm->length[1], &p1, &p2);
    NOTICE("Inverse kinematics: found %d possible solutions", position_count);

    if (position_count == 0) {
        return;
    } else if (position_count == 2) {
        shoulder_mode_t mode;
        mode = scara_orientation_mode(SHOULDER_BACK, 0.);
        p1 = scara_shoulder_solve(target, p1, p2, mode);
    }

    /* p1 now contains the correct elbow pos. */
    float alpha = scara_compute_shoulder_angle(p1, target);
    float beta = scara_compute_elbow_angle(p1, target);

    /* This is due to mecanical construction of the arms. */
    beta = beta - alpha;

    /* The arm cannot make one full turn. */
    if (beta < -M_PI) {
        beta = 2 * M_PI + beta;
    }
    if (beta > M_PI) {
        beta = beta - 2 * M_PI;
    }

    NOTICE("Inverse kinematics: alpha [%.3f] \tbeta [%.3f]", alpha, beta);

    arm->set_shoulder_position(arm->shoulder_dir * alpha);
    arm->set_elbow_position(arm->elbow_dir * beta);
}

void scara_goto_robot(scara_t* arm, float x, float y)
{
    point_t target_robot = {.x = x, .y = y};

    point_t target_arm = scara_coordinate_robot2arm(target_robot, arm->offset_xy, arm->offset_rotation);

    scara_goto_arm(arm, target_arm.x, target_arm.y);
}


void scara_do_trajectory(scara_t *arm, scara_trajectory_t *traj)
{
    // scara_take_semaphore(&arm->trajectory_semaphore);
    scara_trajectory_copy(&arm->trajectory, traj);
    // scara_signal_semaphore(&arm->trajectory_semaphore);
}

void scara_manage(scara_t *arm)
{
    scara_waypoint_t frame;
    point_t target, p1, p2;
    int32_t current_date = scara_time_get();
    int position_count;
    float alpha, beta;

    // scara_take_semaphore(&arm->trajectory_semaphore);

    if (arm->trajectory.frame_count == 0) {
        arm->last_loop = current_date;
        // scara_signal_semaphore(&arm->trajectory_semaphore);
        return;
    }

    frame = scara_position_for_date(arm, scara_time_get());
    target.x = frame.position[0];
    target.y = frame.position[1];

    position_count = scara_num_possible_elbow_positions(target, frame.length[0], frame.length[1], &p1, &p2);

    if (position_count == 0) {
        arm->last_loop = current_date;
        // scara_signal_semaphore(&arm->trajectory_semaphore);
        return;
    } else if (position_count == 2) {
        shoulder_mode_t mode;
        mode = scara_orientation_mode(arm->shoulder_mode, arm->offset_rotation);
        p1 = scara_shoulder_solve(target, p1, p2, mode);
    }

    /* p1 now contains the correct elbow pos. */
    alpha = scara_compute_shoulder_angle(p1, target);
    beta  = scara_compute_elbow_angle(p1, target);
    beta = beta - alpha;

    /* The arm cannot make one full turn. */
    if (beta < -M_PI)
        beta = 2 * M_PI + beta;

    if (beta > M_PI)
        beta = beta - 2 * M_PI;

    arm->last_loop = scara_time_get();
    // scara_signal_semaphore(&arm->trajectory_semaphore);
}

static scara_waypoint_t scara_convert_waypoint_coordinate(scara_t *arm, scara_waypoint_t key)
{
    point_t pos;
    pos.x = key.position[0];
    pos.y = key.position[1];
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

    key.position[0] = pos.x;
    key.position[1] = pos.y;
    return key;
}

scara_waypoint_t scara_position_for_date(scara_t *arm, int32_t date)
{
    int i=0;
    scara_waypoint_t k1, k2;

    /* If we are past last waypoint, simply return last frame. */
    if (arm->trajectory.frames[arm->trajectory.frame_count-1].date < date) {
        k1 = arm->trajectory.frames[arm->trajectory.frame_count-1];
        return scara_convert_waypoint_coordinate(arm, k1);
    }

    while (arm->trajectory.frames[i].date < date)
        i++;

    k1 = scara_convert_waypoint_coordinate(arm, arm->trajectory.frames[i-1]);
    k2 = scara_convert_waypoint_coordinate(arm, arm->trajectory.frames[i]);

    return scara_trajectory_interpolate_waypoints(k1, k2, date);
}

void scara_set_related_robot_pos(scara_t *arm, struct robot_position *pos)
{
    arm->robot_pos = pos;
}

void scara_shutdown(scara_t *arm)
{
    // scara_take_semaphore(&arm->trajectory_semaphore);
    scara_trajectory_delete(&arm->trajectory);
    // scara_signal_semaphore(&arm->trajectory_semaphore);
}
