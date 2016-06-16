#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

void position_manager_start(void);

void position_manager_reset(float x, float y, float heading);

void position_manager_set_wheel_correction(float left, float right);
void position_manager_get_wheel_correction(float *left, float *right);

#ifdef __cplusplus
}
#endif

#endif /* POSITION_MANAGER_H */
