#ifndef STRATEGY_HELPERS_H
#define STRATEGY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "base/base_controller.h"

/** Team color
 */
enum strat_color_t {
    YELLOW = 0,
    BLUE
};


/** Compute the symmetrical position depending on color
 */
#define MIRROR_Y(color, y) (color == YELLOW ? (y) : 2000 - (y))

/** Compute the symmetrical angle depending on color
 */
#define MIRROR_A(color, a) (color == YELLOW ? (a) : -(a))


/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(
        int32_t x, int32_t y, int32_t heading, int32_t robot_size,
        enum strat_color_t robot_color,
        enum board_mode_t* robot_mode,
        struct trajectory* robot_traj,
        struct robot_position* robot_pos,
        struct blocking_detection* robot_distance_blocking,
        struct blocking_detection* robot_angle_blocking);


#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
