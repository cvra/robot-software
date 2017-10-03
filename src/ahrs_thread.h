#ifndef AHRS_THREAD_H
#define AHRS_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q[4]; /**< orientation quaternion */
} attitude_msg_t;

void ahrs_start(void);

#ifdef __cplusplus
}
#endif

#endif
