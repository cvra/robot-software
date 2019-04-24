#ifndef MANIPULATOR_THREAD_H
#define MANIPULATOR_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

void manipulator_start(void);

void manipulator_angles(float* angles);
void manipulator_angles_set(float q1, float q2, float q3);

typedef enum {
    GRIPPER_OFF,
    GRIPPER_ACQUIRE,
    GRIPPER_RELEASE,
} gripper_state_t;

void manipulator_gripper_set(gripper_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MANIPULATOR_THREAD_H */
