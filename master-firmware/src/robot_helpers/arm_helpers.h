#ifndef ARM_HELPERS_H
#define ARM_HELPERS_H

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

typedef enum {
    GRIPPER_OFF,
    GRIPPER_ACQUIRE,
    GRIPPER_RELEASE,
} gripper_state_t;

// Moves the arm motors one after the other to index them.
// Assumes 3 axis. So all pointers must point to C-arrays of length 3.
// Returns the offset positions at which the index was detected for each axis
void arm_motors_index(const char** motors, const float* references, const float* directions, const float* speeds, float* offsets);

// Computes offsets for manual arm indexing
// @param[in] directions direction of the motors to move in positive trigonometric direction
// @param[in, out] offsets inputs are the values measured, will be transformed taking robot geometry into account
void arm_compute_offsets(const float* references, const float* directions, float* offsets);

void arm_manual_index(manipulator_side_t side);

const float RIGHT_ARM_REFS[3] = {
    M_PI_2, // theta 1 indexes at 90deg
    -M_PI, // theta 2 indexes at -180deg from theta 1
    M_PI, // theta 3 indexes at 180deg from theta 2
};
// left is the mirror of the right side
const float LEFT_ARM_REFS[3] = {
    -M_PI_2,
    M_PI,
    -M_PI,
};

#ifdef __cplusplus
}
#endif

#endif /* ARM_HELPERS_H */
