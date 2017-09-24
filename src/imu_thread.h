#ifndef IMU_THREAD_H
#define IMU_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct {
        float x;
        float y;
        float z;
    } gyro; /**< Gyroscope data in rad/s */
    struct {
        float x;
        float y;
        float z;
    } acc; /**< Acceleration data in m/s/s. */
} imu_msg_t;

typedef struct {
    float temperature; /**< Temperature level in degree C. */
} temperature_msg_t;

/** Starts the thread doing the IMU reading. */
void imu_start(void);

#ifdef __cplusplus
}
#endif

#endif
