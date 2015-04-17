#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "unix_timestamp.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        unix_timestamp_t date;
        float val;
    } trajectory_frame_t;

    void trajectory_merge(trajectory_frame_t *traj, int traj_len, trajectory_frame_t *newtraj, int newtraj_len);

#ifdef __cplusplus
}
#endif

#endif
