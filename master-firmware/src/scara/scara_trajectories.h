#ifndef SCARA_TRAJECTORIES_H
#define SCARA_TRAJECTORIES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "scara_waypoint.h"

/** Adds a point to a given trajectory.
 * @param [in, out] traj The trajectory structure to add the point to.
 * @param [in] x,y,z The point to add.
 * @param [in] system The coordinate system of the point.
 * @param [in] max_vel The maximum velocity for the trajectory segment between
 *             this point and the previous one.
 *
 * @note This function tests if the given trajectory is empty, and if it is, it assumes the first point
 * date is now.
 */
void scara_trajectory_append_point(scara_trajectory_t* traj, position_3d_t pos, scara_coordinate_t system, velocity_3d_t max_vel, const float* length);

/** Zeroes an scara_trajectory_t structure to avoid problems.
 * @param traj The trajectory to zero.
 */
void scara_trajectory_init(scara_trajectory_t* traj);

void scara_trajectory_delete(scara_trajectory_t* traj);

void scara_trajectory_copy(scara_trajectory_t* dest, scara_trajectory_t* src);

int scara_trajectory_finished(scara_trajectory_t* traj);

bool scara_trajectory_is_empty(scara_trajectory_t* trajectory);

/** Interpolates two waypoint.
 * @warning All frames must be expressed in arm frame. Otherwise invalid results
 * will be yielded.
 */
scara_waypoint_t scara_trajectory_interpolate_waypoints(scara_waypoint_t k1, scara_waypoint_t k2, int32_t date);

/** Compute trajectory duration based on slowest axis */
float scara_trajectory_duration(position_3d_t start, position_3d_t end, velocity_3d_t max_vel);

#ifdef __cplusplus
}
#endif

#endif /* SCARA_TRAJECTORIES_H */
