#include <ch.h>
#include <string.h>
#include "trajectories.h"
#include "usbconf.h"
#include <chprintf.h>
#include <hal.h>

void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us)
{
    traj->buffer = buffer;
    traj->length = len;
    traj->dimension = dimension;
    traj->sampling_time_us = sampling_time_us;
    traj->read_pointer = 0;
    traj->read_time_us = traj->last_defined_time_us = 0;
    traj->last_chunk_start_time_us = 0;
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

    if (chunk->sampling_time_us != traj->sampling_time_us) {
        return TRAJECTORY_ERROR_TIMESTEP_MISMATCH;
    }

    if (chunk->dimension != traj->dimension) {
        return TRAJECTORY_ERROR_DIMENSION_MISMATCH;
    }

    if (chunk->start_time_us < traj->last_chunk_start_time_us) {
        return TRAJECTORY_ERROR_CHUNK_OUT_OF_ORER;
    }

    /* Check if the end of the chunk is past the end of the trajectory. */
    if (chunk->start_time_us + chunk->length * chunk->sampling_time_us >=
        traj->read_time_us + traj->length * traj->sampling_time_us) {
        return TRAJECTORY_ERROR_CHUNK_TOO_LONG;
    }

    traj->last_chunk_start_time_us = chunk->start_time_us;

    start_index = (chunk->start_time_us - traj->read_time_us) / (traj->sampling_time_us);
    start_index += traj -> read_pointer;

    /* Do not copy points before the read pointer. */
    i = 1 + ((traj->read_time_us - chunk->start_time_us) / traj->sampling_time_us);

    if (i < 0) {
        i = 0;
    }


    while (i < chunk->length) {
        write_index = (i + start_index) % traj->length;

        memcpy(&traj->buffer[write_index * traj->dimension],
               &chunk->buffer[i * traj->dimension],
               traj->dimension * sizeof (float));

        i ++;
    }
    traj->last_defined_time_us = chunk->start_time_us +
                          (chunk->length - 1) * chunk->sampling_time_us;
    return 0;
}

float* trajectory_read(trajectory_t *traj, int64_t time)
{
    int64_t read_time = traj->read_time_us;
    int offset = (time - read_time + 0.5 * traj->sampling_time_us) / traj->sampling_time_us;

    traj->read_time_us = time;
    if (time > traj->last_defined_time_us) {
        return NULL;
    }

    if (time < read_time) {
        return NULL;
    }

    //traj->read_time_us += offset * traj->sampling_time_us;
    //

    traj->read_pointer += offset;
    traj->read_pointer = traj->read_pointer % traj->length;


    return &traj->buffer[traj->read_pointer * traj->dimension];
}
