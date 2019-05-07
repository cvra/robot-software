#ifndef ROBOT_HELPERS_ROBOT_H
#define ROBOT_HELPERS_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RIGHT = (0b01),
    LEFT = (0b10),
    BOTH = (0b11),
} manipulator_side_t;
#define USE_RIGHT(side) ((side & RIGHT) > 0)
#define USE_LEFT(side) ((side & LEFT) > 0)

#define MIRROR_ARM(side, x) ((side == RIGHT) ? (+x) : (-x))

typedef enum {
    GRIPPER_OFF,
    GRIPPER_ACQUIRE,
    GRIPPER_RELEASE,
} gripper_state_t;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_HELPERS_ROBOT_H */
