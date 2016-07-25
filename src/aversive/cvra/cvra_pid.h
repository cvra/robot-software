#ifndef CVRA_PID_H
#define CVRA_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pid/pid.h"

typedef struct {
    pid_ctrl_t pid;
    float divider;
} cvra_pid_t;

void cvra_pid_set_out_divider(void *pid, float divider);
int32_t cvra_pid_process(void *pid, int32_t error);


#ifdef __cplusplus
}
#endif

#endif /* CVRA_PID_H */
