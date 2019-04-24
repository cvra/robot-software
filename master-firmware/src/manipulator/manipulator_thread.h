#ifndef MANIPULATOR_THREAD_H
#define MANIPULATOR_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

void manipulator_start(void);

void manipulator_angles(float* angles);
void manipulator_angles_set(float* angles);

#ifdef __cplusplus
}
#endif

#endif /* MANIPULATOR_THREAD_H */
