#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid/pid.h"
#include "base.h"


typedef struct {
    pid_ctrl_t distance_pid;
    pid_ctrl_t heading_pid;
} base_controller_t;

void base_controller_init(base_controller_t *base);

void base_controller_compute_error(polar_t *error, pose2d_t desired, pose2d_t measured);


#ifdef __cplusplus
}
#endif

#endif /* BASE_CONTROLLER_H */
