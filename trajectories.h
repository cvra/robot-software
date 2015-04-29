#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float *buffer;
    int length;
    int dimension;
    uint64_t sampling_time_us;
} trajectory_t;

/** Inits a trajectory structure.
 *
 * @param [in] traj The trajectory to initialize.
 * @param [in] buffer The buffer to use for the trajectory.
 * @param [in] len The max number of points in the trajectory.
 * @param [in] dimension The dimension of a point in the trajectory.
 * @param [in] sampling_time_us Time between 2 samples in us.
 */
void trajectory_init(trajectory_t *traj,
                     float *buffer, int len, int dimension,
                     uint64_t sampling_time_us);

#ifdef __cplusplus
}
#endif

#endif
