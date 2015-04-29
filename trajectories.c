#include "trajectories.h"

void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us)
{
    traj->buffer = buffer;
    traj->length = len;
    traj->dimension = dimension;
    traj->sampling_time_us = sampling_time_us;
}
