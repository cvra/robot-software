#ifndef IMU_THREAD_H
#define IMU_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu9250.h"

/** Starts the thread doing the IMU reading. */
void imu_start(void);

#ifdef __cplusplus
}
#endif

#endif
