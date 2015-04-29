#include "trajectories.h"

void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us)
{
    traj->buffer = buffer;
    traj->length = len;
    traj->dimension = dimension;
    traj->sampling_time_us = sampling_time_us;
    traj->read_pointer = 0;
}

void trajectory_chunk_init(trajectory_chunk_t *chunk, float *buffer, int length,
                           int dimension, uint64_t start_time_us,
                           uint64_t sampling_time_us)
{
    chunk->buffer = buffer;
    chunk->length = length;
    chunk->dimension = dimension;
    chunk->sampling_time_us = sampling_time_us;
    chunk->start_time_us = start_time_us;
}
