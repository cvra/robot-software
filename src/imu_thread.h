#ifndef IMU_THREAD_H
#define IMU_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t timestamp; /**< Timestamp in us since boot. */
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
    uint32_t timestamp;/**< Timestamp in us since boot. */
    float temperature; /**< Temperature level in degree C. */
} temperature_msg_t;

/** Starts the thread doing the IMU reading. */
void imu_start(void);

#ifdef __cplusplus
}
#endif

#endif
