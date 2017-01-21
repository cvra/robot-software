#include <string.h>
#include <math.h>

#include "scara.h"
#include "scara_kinematics.h"
#include "scara_trajectories.h"
#include "scara_utils.h"
#include "scara_port.h"


void scara_set_physical_parameters(scara_t *arm)
{
    /* Physical constants, not magic numbers. */
    arm->length[0] = 135.16; /* mm */
    arm->length[1] = 106.5;
}


void scara_init(scara_t *arm)
{
    memset(arm, 0, sizeof(scara_t));

    // scara_cs_init_loop(&arm->shoulder);
    // scara_cs_init_loop(&arm->hand);
    // scara_cs_init_loop(&arm->elbow);
    // scara_cs_init_loop(&arm->z_axis);

    /* Sets last loop run date for lag compensation. */
    arm->last_loop = scara_time_get();

    arm->shoulder_mode = SHOULDER_BACK;

    // platform_create_semaphore(&arm->trajectory_semaphore, 1);
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
        // cs_disable(&arm->shoulder.manager);
        // cs_disable(&arm->elbow.manager);
        // cs_disable(&arm->z_axis.manager);
        // cs_disable(&arm->hand.manager);
        arm->last_loop = current_date;
        // scara_signal_semaphore(&arm->trajectory_semaphore);
        return;
    }

    frame = scara_position_for_date(arm, scara_time_get());
    target.x = frame.position[0];
    target.y = frame.position[1];

    position_count = scara_num_possible_elbow_positions(target, frame.length[0], frame.length[1], &p1, &p2);

    if (position_count == 0) {
        // cs_disable(&arm->shoulder.manager);
        // cs_disable(&arm->elbow.manager);
        // cs_disable(&arm->z_axis.manager);
        // cs_disable(&arm->hand.manager);
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


    /* This is due to mecanical construction of the arms. */
    beta = beta - alpha;


    /* The arm cannot make one full turn. */

    if (beta < -M_PI)
        beta = 2 * M_PI + beta;

    if (beta > M_PI)
        beta = beta - 2 * M_PI;

    // cs_enable(&arm->shoulder.manager);
    // cs_enable(&arm->elbow.manager);
    // cs_enable(&arm->z_axis.manager);
    // cs_enable(&arm->hand.manager);

    // cs_set_consign(&arm->shoulder.manager, alpha * arm->shoulder_imp_per_rad);
    // cs_set_consign(&arm->elbow.manager, beta * arm->elbow_imp_per_rad);
    // cs_set_consign(&arm->z_axis.manager, frame.position[2] * arm->z_axis_imp_per_mm);
    // cs_set_consign(&arm->hand.manager, frame.hand_angle * arm->hand_imp_per_deg);

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
