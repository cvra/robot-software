#ifndef TRAJECTORY_HELPERS_H
#define TRAJECTORY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <msgbus/messagebus.h>

#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "base/base_controller.h"

/** Duration of a game in seconds. */
#define GAME_DURATION 100

#define TRAJ_MIN_DISTANCE_TO_OPPONENT           0.6  // we stop if there is 60cm or less to opponent
#define TRAJ_MIN_DIRECTION_TO_OPPONENT          0.5  // defines cone in which to consider opponents (cone is double the angle in size)
#define TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION  0.25 // if delay bigger that this, beacon signal is discarded
#define TRAJ_MAX_TIME_DELAY_ALLY_DETECTION      0.5 // if delay bigger that this, ally position is discarded

#define TRAJ_END_GOAL_REACHED   (1 << 0)
#define TRAJ_END_COLLISION      (1 << 1)
#define TRAJ_END_OPPONENT_NEAR  (1 << 2)
#define TRAJ_END_TIMER          (1 << 3)

#define TRAJ_FLAGS_ALL (TRAJ_END_GOAL_REACHED | TRAJ_END_COLLISION | TRAJ_END_OPPONENT_NEAR | \
                        TRAJ_END_TIMER)

#define TRAJ_FLAGS_SHORT_DISTANCE (TRAJ_END_GOAL_REACHED | TRAJ_END_TIMER)
#define TRAJ_FLAGS_ROTATION (TRAJ_END_GOAL_REACHED | TRAJ_END_COLLISION | TRAJ_END_TIMER)

/** Returns when ongoing trajectory is finished for the reasons specified
 *  For example when goal is reached
 * @note This is a blocking function call
 * @warning Will not return if you misspecify the reasons to watch
 *      (ie. the reason watched never occurs)
 *
 * @param watched_end_reasons bitmask of the end reasons to watch for
 */
int trajectory_wait_for_end(int watched_end_reasons);

/** Watches the robot state for the reasons specified
 *  Returns with the end reason of the trajectory if watching it
 *  Otherwise returns 0
 *
 * @param watched_end_reasons bitmask of the end reasons to watch for
 */
int trajectory_has_ended(int watched_end_reasons);

/** Go backwards until a wall is hit to align with it
 */
void trajectory_align_with_wall(void);

/** Go to request (x, y, a) point on table
 * @note This is a blocking call that returns when the goal is reached
 */
void trajectory_move_to(int32_t x_mm, int32_t y_mm, int32_t a_deg);

/** Check if current trajectory segment crosses the passed obstacle
 */
bool trajectory_crosses_obstacle(poly_t* opponent, point_t* intersection);

/** Check if the current trajectory will collide with the obstacle
    seen at position (x,y)
 */
bool trajectory_is_on_collision_path(int x, int y);

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

/** Set game starting time
 */
void trajectory_game_timer_reset(void);

/** Get current game time
 */
int trajectory_get_time(void);

/** Tell if time is up
 */
bool trajectory_game_has_ended(void);


#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_HELPERS_H */
