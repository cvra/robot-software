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
#define MIRROR_X(color, x) (color == YELLOW ? (x) : 3000 - (x))

/** Initialize map with obstacles */
void strategy_map_setup(int32_t robot_size);

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(
        int32_t x, int32_t y, int32_t heading, int32_t robot_size,
        enum strat_color_t robot_color, struct _robot* robot);


#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
