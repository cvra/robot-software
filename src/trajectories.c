#include "trajectories.h"
#include <stdio.h>


void trajectory_merge(trajectory_frame_t *traj, int traj_len,
                      trajectory_frame_t *newtraj, int newtraj_len)
{
    int write_index = 0;

    while (timestamp_unix_compare(traj[write_index].date, newtraj[0].date) > 0) {
        write_index ++;
    }

    while (timestamp_unix_compare(traj[write_index].date, newtraj[0].date) < 0) {
        write_index ++;
    }

    for (int i = 0; i < newtraj_len; ++i) {
        traj[(write_index + i) % traj_len].val = newtraj[i].val;
    }
}
