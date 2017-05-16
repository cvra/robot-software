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
    YELLOW=0,
    BLUE
};

typedef struct {
    float x; // Desired position x of end effector in mm without mirroring
    float y; // Desired position y of end effector in mm without mirroring
    float z; // Desired position z of end effector in mm without mirroring
    float a; // Desired position a of end effector in deg without mirroring
    float p; // Desired position p of end effector in deg without mirroring
    scara_coordinate_t coord; // Coordinate system
    unsigned dt; // Duration in ms to reach this waypoint
    float l3; // Length from wrist to end effector
} arm_waypoint_t;


/** Compute the symmetrical position depending on color
 */
#define MIRROR_X(color, x) (color == YELLOW ? (x) : 3000 - (x))
#define MIRROR_A(color, a_deg) (color == YELLOW ? (a_deg) : 180 - (a_deg))

/** Auto position robot at requested location, and ensure the correct
 *  position is reached by aligning against walls.
 */
void strategy_auto_position(int32_t x, int32_t y, int32_t heading, enum strat_color_t robot_color);

/** Align on y axis */
void strategy_align_y(int32_t y);

/** Make and follow a given set of arm waypoints with mirroring
 * Returns the duration of the trajectory
 */
unsigned strategy_set_arm_trajectory(scara_t* arm, enum strat_color_t color, arm_waypoint_t* trajectory, unsigned trajectory_length);


#ifdef __cplusplus
}
#endif

#endif /* STRATEGY_HELPERS_H */
