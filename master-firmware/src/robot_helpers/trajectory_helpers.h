#ifndef TRAJECTORY_HELPERS_H
#define TRAJECTORY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"

/** Return when ongoing trajectory is finished (ie. goal reached)
 * @note This is a blocking function call
 */
void trajectory_wait_for_finish(struct trajectory* robot_traj);

/** Return when robot collides with something (ie. wall/opponent hit)
 * @note This is a blocking function call
 */
void trajectory_wait_for_collision(struct blocking_detection* distance_blocking);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_HELPERS_H */
