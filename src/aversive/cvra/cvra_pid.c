#include "cvra/cvra_pid.h"

void cvra_pid_set_out_divider(void *_pid, float divider)
{
    cvra_pid_t *pid = (cvra_pid_t *)_pid;
    pid->divider = divider;
}

int32_t cvra_pid_process(void *_pid, int32_t error)
{
    cvra_pid_t *pid = (cvra_pid_t *)_pid;
    float cmd = pid_process(&(pid->pid), (float)error / pid->divider);
    return cmd * pid->divider;
}
