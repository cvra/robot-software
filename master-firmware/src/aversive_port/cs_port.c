#include "cs_port.h"

void cs_pid_set_out_divider(void* _pid, float divider)
{
    cs_pid_t* pid = (cs_pid_t*)_pid;
    pid->divider = divider;
}

int32_t cs_pid_process(void* _pid, int32_t error)
{
    cs_pid_t* pid = (cs_pid_t*)_pid;
    float cmd = pid_process(&(pid->pid), (float)error / pid->divider);
    return cmd * pid->divider;
}
