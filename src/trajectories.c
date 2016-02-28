#include <string.h>
#include "trajectories.h"
#include <assert.h>
#include "log.h"

#include <stdio.h>

void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us)
{
    traj->buffer = buffer;
    traj->length = len;
    traj->dimension = dimension;
    traj->sampling_time_us = sampling_time_us;
    traj->read_index = 0;
    traj->read_time_us = traj->last_defined_time_us = 0;
    traj->last_chunk_start_time_us = 0;
}


void *trajectory_get_buffer_pointer(trajectory_t *traj)
{
    return traj->buffer;
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


static int min(int a, int b)
{
    if (a < b) {
        return a;
    }
    return b;
}

int _trajectory_copy_from_buffer(trajectory_t *traj,
                                  int64_t write_time_us,
                                  const float *buffer,
                                  int nb_points)
{
    if (nb_points == 0) {
        return 0;
    }

    int write_offset_to_read_index = (write_time_us - traj->read_time_us) / traj->sampling_time_us;
    int write_index = traj->read_index + write_offset_to_read_index;
    int nb_points_free = traj->length - write_offset_to_read_index;
    assert(nb_points_free > 0);     // the calling code asserts this
    int nb_points_to_copy = min(nb_points, nb_points_free);

    write_index %= traj->length;
    int nb_points_to_end = traj->length - write_index;
    int nb_points_to_copy_till_buf_end = min(nb_points_to_copy, nb_points_to_end);
    int nb_points_to_copy_wrapped = nb_points_to_copy - nb_points_to_copy_till_buf_end;

    memcpy(&traj->buffer[write_index * traj->dimension],
           buffer,
           nb_points_to_copy_till_buf_end * traj->dimension * sizeof(float));

    memcpy(&traj->buffer[0],
           &buffer[nb_points_to_copy_till_buf_end * traj->dimension],
           nb_points_to_copy_wrapped * traj->dimension * sizeof(float));

    int64_t last_point_offset_to_read_index = (write_offset_to_read_index + nb_points_to_copy - 1);
    traj->last_defined_time_us = traj->read_time_us + last_point_offset_to_read_index * traj->sampling_time_us;

    return 0;
}

int trajectory_apply_chunk(trajectory_t *traj, const trajectory_chunk_t *chunk)
{
    if (chunk->sampling_time_us != traj->sampling_time_us) {
        return TRAJECTORY_ERROR_TIMESTEP_MISMATCH;
    }

    if (chunk->dimension != traj->dimension) {
        return TRAJECTORY_ERROR_DIMENSION_MISMATCH;
    }

    if (chunk->start_time_us < traj->last_chunk_start_time_us) {
        return TRAJECTORY_ERROR_CHUNK_OUT_OF_ORER;
    }


    traj->last_chunk_start_time_us = chunk->start_time_us;

    // trajectories are sent with overlap
    if (traj->last_defined_time_us < chunk->start_time_us) {
        log_message("WARNING: trajectroy apply chunk: last defined < chunk start -> reset traj");
        traj->read_index = 0;
        traj->read_time_us = chunk->start_time_us;
    }


    int64_t write_time_us;
    float *buffer;
    int first_chunk_point_idx;
    int nb_points_to_copy;

    if (chunk->start_time_us >= traj->read_time_us) {
        write_time_us = chunk->start_time_us;
    } else {
        write_time_us = traj->read_time_us + traj->sampling_time_us;
    }

    /* chunks which are too far in the future to overlap the currently defined
     * tarjectory reset the trajectory to the current chunk.
     * Therefore, the write_time is always inside the trajectory buffer. */
    assert(write_time_us < traj->read_time_us + traj->length * traj->sampling_time_us);


    first_chunk_point_idx = (write_time_us - chunk->start_time_us) / traj->sampling_time_us;
    assert(first_chunk_point_idx >= 0);
    buffer = &chunk->buffer[first_chunk_point_idx * traj->dimension];
    nb_points_to_copy = chunk->length - first_chunk_point_idx;

    if (nb_points_to_copy <= 0) {
        return TRAJECTORY_ERROR_CHUNK_TOO_OLD;
    }

    return _trajectory_copy_from_buffer(traj, write_time_us, buffer, nb_points_to_copy);
}

float* trajectory_read(trajectory_t *traj, int64_t time)
{
    int64_t read_time = traj->read_time_us;
    int offset = (time - read_time + 0.5 * traj->sampling_time_us) / traj->sampling_time_us;

    if (time > traj->last_defined_time_us) {
        return NULL;
    }

    if (time < read_time) {
        return NULL;
    }

    traj->read_time_us += offset * traj->sampling_time_us;

    traj->read_index += offset;
    traj->read_index = traj->read_index % traj->length;


    return &traj->buffer[traj->read_index * traj->dimension];
}
