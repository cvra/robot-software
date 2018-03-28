#ifndef STRATEGY_HELPERS_H
#define STRATEGY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <msgbus/messagebus.h>
#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "base/base_controller.h"
#include "math/lie_groups.h"

/** Team color
 */
enum strat_color_t {
    YELLOW=0,
    BLUE
};

/** Blocks color */
enum block_color {
    BLOCK_YELLOW = 0, // ( 0,  0)
    BLOCK_GREEN,      // ( l,  0)
    BLOCK_BLUE,       // ( 0,  l)
    BLOCK_RED,        // (-l,  0)
    BLOCK_BLACK,      // ( 0, -l)
};

/** Lever side */
enum lever_side_t {
    LEVER_SIDE_LEFT,
    LEVER_SIDE_RIGHT,
};

/** Compute the symmetrical position depending on color
 */
#define MIRROR_X(color, x) (color == YELLOW ? (x) : 3000. - (x))
#define MIRROR_A(color, a_deg) (color == YELLOW ? (a_deg) : 180. - (a_deg))
#define MIRROR(color, value) (color == YELLOW ? (value) : - (value))

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(int32_t x, int32_t y, int32_t heading, enum strat_color_t robot_color);

/** Align on y axis */
void strategy_align_y(int32_t y);

/** Compute block position given blocks pose in table frame and block color*/
point_t strategy_block_pos(se2_t blocks_pose, enum block_color color);

#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
