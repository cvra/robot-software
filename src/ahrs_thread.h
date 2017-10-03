#ifndef AHRS_THREAD_H
#define AHRS_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t timestamp; /**< Timestamp in us since boot. */
    struct {
        float w;
        float x;
        float y;
        float z;
    } q; /**< orientation quaternion */
} attitude_msg_t;

void ahrs_start(void);

void ahrs_calibrate_gyro(void);

#ifdef __cplusplus
}
#endif

#endif
