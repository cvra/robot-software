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

/** Returns when ongoing trajectory is finished for the reasons specified
 *  For example when goal is reached
 * @note This is a blocking function call
 * @warning Will not return if you misspecify the reasons to watch
 *      (ie. the reason watched never occurs)
 *
 * @param watched_end_reasons bitmask of the end reasons to watch for
 */
int trajectory_wait_for_end(struct _robot *robot, int watched_end_reasons);

/** Watches the robot state for the reasons specified
 *  Returns with the end reason of the trajectory if watching it
 *  Otherwise returns 0
 *
 * @param watched_end_reasons bitmask of the end reasons to watch for
 */
int trajectory_has_ended(struct _robot *robot, int watched_end_reasons);

/** Go backwards until a wall is hit to align with it
 */
void trajectory_align_with_wall(struct _robot *robot);

/** Go to request (x, y, a) point on table
 * @note This is a blocking call that returns when the goal is reached
 */
void trajectory_move_to(struct _robot* robot, int32_t x_mm, int32_t y_mm, int32_t a_deg);

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
