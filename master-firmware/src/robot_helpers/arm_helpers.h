#ifndef ARM_HELPERS_H
#define ARM_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

// Moves the arm motors one after the other to index them.
// Assumes 3 axis. So all pointers must point to C-arrays of length 3.
// Returns the offset positions at which the index was detected for each axis
void arm_motors_index(const char** motors, const float* directions, const float* speeds, float* offsets);

#ifdef __cplusplus
}
#endif

#endif /* ARM_HELPERS_H */
