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

void trajectory_apply_chunk(trajectory_t *traj, trajectory_chunk_t *chunk)
{
    int start_index, i = 0, write_index, j;
    int64_t start_time_us;


    start_index = (chunk->start_time_us - traj->read_time_us) / (traj->sampling_time_us);
    start_index += traj -> read_pointer;

    start_time_us = chunk->start_time_us;

    /* If the beginning of the chunk is before the read time, skip the first
     * points. */
    while (start_time_us <= traj->read_time_us) {
        start_time_us += traj->sampling_time_us;
        i ++;
    }

    while (i < chunk->length) {
        write_index = (i + start_index) % traj->length;

        for (j = 0; j < traj->dimension; ++j) {
            traj->buffer[write_index * traj->dimension + j] =
                chunk->buffer[i * traj->dimension + j];
        }

        i ++;
    }
}

float* trajectory_read(trajectory_t *traj, int64_t time)
{
    while (traj->read_time_us < time) {
        traj->read_time_us += traj->sampling_time_us;
        traj->read_pointer ++;
    }

    traj->read_pointer = traj->read_pointer % traj->length;
    return &traj->buffer[traj->read_pointer * traj->dimension];
}
