#ifndef ARM_HELPERS_H
#define ARM_HELPERS_H

#include "robot_helpers/robot.h"

#ifdef __cplusplus
extern "C" {
#endif

// Moves the arm motors one after the other to index them.
// Assumes 3 axis. So all pointers must point to C-arrays of length 3.
// Returns the offset positions at which the index was detected for each axis
void arm_motors_index(const char** motors, const float* references, const float* directions, const float* speeds, float* offsets);

// Computes offsets for manual arm indexing
// @param[in] directions direction of the motors to move in positive trigonometric direction
// @param[in, out] offsets inputs are the values measured, will be transformed taking robot geometry into account
void arm_compute_offsets(const float* references, const float* directions, float* offsets);

void arm_manual_index(manipulator_side_t side);

#ifdef __cplusplus
}
#endif

#endif /* ARM_HELPERS_H */
