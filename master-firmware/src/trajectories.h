#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TRAJECTORY_ERROR_TIMESTEP_MISMATCH          -1
#define TRAJECTORY_ERROR_CHUNK_TOO_OLD              -2
#define TRAJECTORY_ERROR_DIMENSION_MISMATCH         -3
#define TRAJECTORY_ERROR_CHUNK_OUT_OF_ORER          -4

typedef struct {
    float *buffer;
    int length;
    int dimension;
    int64_t sampling_time_us;
    int read_index;
    int64_t read_time_us;
    int64_t last_defined_time_us;
    int64_t last_chunk_start_time_us; /**< For out of order arrival detection. */
} trajectory_t;

typedef struct {
    float *buffer;
    int length;
    int dimension;
    int64_t sampling_time_us;
    int64_t start_time_us;

} trajectory_chunk_t;


/** Inits a trajectory structure.
 *
 * @param [in] traj The trajectory to initialize.
 * @param [in] buffer The buffer to use for the trajectory.
 * @param [in] len The max number of points in the trajectory.
 * @param [in] dimension The dimension of a point in the trajectory.
 * @param [in] sampling_time_us Time between 2 samples in us.
 * @param [in] start_time_us Starting time in us.
 */
void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us);


/** this is used to free the trajectory buffer
 *
 * @param [in] traj trajectory pointer
 */
void *trajectory_get_buffer_pointer(trajectory_t *traj);


/** Inits a trajectory chunk. A chunk is a part of a trajectory that can later
 * be merged to a trajectory.
 *
 * @param [in] chunk The chunk to initialize.
 * @param [in] buffer The buffer to use for the chunk.
 * @param [in] len The max number of points in the chunk.
 * @param [in] dimension The dimension of a point in the chunk.
 * @param [in] sampling_time_us Time between 2 samples in us.
 * @param [in] start_time_us Starting time in us.
 */
void trajectory_chunk_init(trajectory_chunk_t *chunk, float *buffer, int length,
                           int dimension, uint64_t start_time_us,
                           uint64_t sampling_time_us);


/** Merges the given trajectory with the given chunk.
 *
 * @returns 0 if everything was OK.
 * @returns TRAJECTORY_ERROR_TIMESTEP_MISMATCH If the timestep from the
 * trajectory is not the same as the chunk's one.
 * @returns TRAJECTORY_ERROR_CHUNK_TOO_LONG if the chunk is too long or too far
 * in the future to be applied.
 * @returns TRAJECTORY_ERROR_DIMENSION_MISMATCH if the chunk's points do not
 * have the same dimesnion as the trajectory's ones.
 *
 * @note If an error occurs, the trajectory is left unchanged.
 */
int trajectory_apply_chunk(trajectory_t *traj, const trajectory_chunk_t *chunk);

/** Reads the current point of the trajectory.
 *
 * @returns A pointer to the first field of the current point.
 * @note This updates the trajectory, settting the read pointer and read time.
 */
float* trajectory_read(trajectory_t *traj, int64_t time);


#ifdef __cplusplus
}
#endif

#endif
