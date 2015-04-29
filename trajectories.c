#include <string.h>
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

int trajectory_apply_chunk(trajectory_t *traj, trajectory_chunk_t *chunk)
{
    int start_index, i = 0, write_index;
    int64_t start_time_us;
    float *src, *dst;

    if (chunk->sampling_time_us != traj->sampling_time_us) {
        return TRAJECTORY_ERROR_TIMESTEP_MISMATCH;
    }

    /* Check if the end of the chunk is past the end of the trajectory. */
    if (chunk->start_time_us + chunk->length * chunk->sampling_time_us >=
        traj->read_time_us + traj->length * traj->sampling_time_us) {
        return TRAJECTORY_ERROR_CHUNK_TOO_LONG;
    }

    start_index = (chunk->start_time_us - traj->read_time_us) / (traj->sampling_time_us);
    start_index += traj -> read_pointer;

    start_time_us = chunk->start_time_us;

    /* Do not copy points before the read pointer. */
    i = 1 + ((traj->read_time_us - chunk->start_time_us) / traj->sampling_time_us);

    while (i < chunk->length) {
        write_index = (i + start_index) % traj->length;

        memcpy(&traj->buffer[write_index * traj->dimension],
               &chunk->buffer[i * traj->dimension],
               traj->dimension * sizeof (float));

        i ++;
    }
    return 0;
}

float* trajectory_read(trajectory_t *traj, int64_t time)
{
    traj->read_pointer += (time - traj->read_time_us) / traj->sampling_time_us;
    traj->read_pointer = traj->read_pointer % traj->length;

    traj->read_time_us = time;
    return &traj->buffer[traj->read_pointer * traj->dimension];
}
