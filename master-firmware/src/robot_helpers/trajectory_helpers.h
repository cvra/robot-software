#ifndef TRAJECTORY_HELPERS_H
#define TRAJECTORY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "base/base_controller.h"

#define TRAJ_END_GOAL_REACHED   (1 << 0)
#define TRAJ_END_COLLISION      (1 << 1)

/** Return when ongoing trajectory is finished (ie. goal reached)
 * @note This is a blocking function call
 */
void trajectory_wait_for_finish(struct trajectory* robot_traj);

/** Return when robot collides with something (ie. wall/opponent hit)
 * @note This is a blocking function call
 */
void trajectory_wait_for_collision(struct blocking_detection* distance_blocking);


int trajectory_has_ended(struct _robot *robot, int end_reason);

/** Go backwards until a wall is hit to aligne with it
 */
void trajectory_align_with_wall(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking);

/** Go to request (x, y, a) point on table
 * @note This is a blocking call that returns when the goal is reached
 */
void trajectory_move_to(struct trajectory* robot_traj, int32_t x_mm, int32_t y_mm, int32_t a_deg);

/** Prepare robot for aligning by settings its dynamics accordingly
 * ie. slower and less sensitive to collisions
 */
void trajectory_set_mode_aligning(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking);

/** Prepare robot for game by settings its dynamics accordingly
 */
void trajectory_set_mode_game(
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct blocking_detection* distance_blocking,
        struct blocking_detection* angle_blocking);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_HELPERS_H */
