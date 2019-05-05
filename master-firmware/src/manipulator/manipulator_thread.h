#ifndef MANIPULATOR_THREAD_H
#define MANIPULATOR_THREAD_H

#include "manipulator/path.h"
#include "robot_helpers/arm_helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MANIPULATOR_DEFAULT_TIMEOUT_MS 2000

void manipulator_start(void);

void manipulator_angles(manipulator_side_t side, float* angles);
void manipulator_angles_set(manipulator_side_t side, float q1, float q2, float q3);
void manipulator_angles_wait_for_traj_end(manipulator_side_t side, uint16_t timeout_ms);
void manipulator_angles_wait_for_traj_end_near(manipulator_side_t side, uint16_t timeout_ms);

// Set angles and wait until motion finished
void manipulator_angles_goto_timeout(manipulator_side_t side, float q1, float q2, float q3, uint16_t timeout_ms);

// go to a define arm position, going through necessary intermediate positions
bool manipulator_goto(manipulator_side_t side, manipulator_state_t target);

void manipulator_gripper_set(manipulator_side_t side, gripper_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MANIPULATOR_THREAD_H */
