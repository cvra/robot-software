#ifndef STRATEGY_HELPERS_H
#define STRATEGY_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <msgbus/messagebus.h>
#include "trajectory_manager/trajectory_manager.h"
#include "blocking_detection_manager/blocking_detection_manager.h"
#include "base/base_controller.h"
#include "scara/scara.h"

/** Team color
 */
enum strat_color_t {
    YELLOW = 0,
    BLUE
};


/** Compute the symmetrical position depending on color
 */
#define MIRROR_X(color, x) (color == YELLOW ? (x) : 3000 - (x))

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(
        int32_t x, int32_t y, int32_t heading, int32_t robot_size,
        enum strat_color_t robot_color, struct _robot* robot, messagebus_t* bus);

/** Move the arm in table coordinates */
void strategy_arm_goto(struct _robot* robot, scara_t* arm, float x, float y, float a_deg);


#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
