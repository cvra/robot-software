#ifndef SCARA_TRAJECTORIES_H
#define SCARA_TRAJECTORIES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "scara_waypoint.h"

/** Adds a point to a given trajectory.
 * @param [in, out] traj The trajectory structure to add the point to.
 * @param [in] x,y,z The point to add.
 * @param [in] system The coordinate system of the point.
 * @param [in] duration The time between this point and the previous one in second.
 *
 * @note This function tests if the given trajectory is empty, and if it is, it assumes the first point
 * date is now.
 */
void scara_trajectory_append_point(scara_trajectory_t *traj, const float x, const float y, const float z, const float a,
                                   const float p, scara_coordinate_t system, const float duration, const float* length);

/** Zeroes an scara_trajectory_t structure to avoid problems.
 * @param traj The trajectory to zero.
 */
void scara_trajectory_init(scara_trajectory_t *traj);


/** same as scara_trajectory_append_point but with a custom length. */
void scara_trajectory_append_point_with_length(scara_trajectory_t *traj, const float x, const float y, const float z,
                                               const float a, const float p, scara_coordinate_t system,
                                               const float duration, const float l1, const float l2, const float l3);

void scara_trajectory_delete(scara_trajectory_t *traj);

void scara_trajectory_copy(scara_trajectory_t *dest, scara_trajectory_t *src);

int scara_trajectory_finished(scara_trajectory_t *traj);

/** Interpolates two waypoint.
 * @warning All frames must be expressed in arm frame. Otherwise invalid results
 * will be yielded.
 */
scara_waypoint_t scara_trajectory_interpolate_waypoints(scara_waypoint_t k1, scara_waypoint_t k2, int32_t date);


#ifdef __cplusplus
}
#endif

#endif /* SCARA_TRAJECTORIES_H */
