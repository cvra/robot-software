#include "trajectories.h"
#include <stdio.h>


void trajectory_merge(trajectory_frame_t *traj, int traj_len,
                      trajectory_frame_t *newtraj, int newtraj_len)
{
    int write_index, i;

    write_index = trajectory_find_point_after(traj, traj_len, newtraj[0].date);

    for (i = 0; i < newtraj_len; ++i) {
        traj[(write_index + i) % traj_len].val = newtraj[i].val;
        traj[(write_index + i) % traj_len].date.s = newtraj[i].date.s;
        traj[(write_index + i) % traj_len].date.us = newtraj[i].date.us;
    }
}

int trajectory_find_point_after(trajectory_frame_t *traj, int traj_len,
                                unix_timestamp_t date)
{
    int index = 0;

    while (timestamp_unix_compare(traj[index].date, date) > 0) {
        index ++;
        if (index >= traj_len) {
            /* Given date is before the earliest point in trajectory. */
            return -1;
        }
    }

    while (timestamp_unix_compare(traj[index % traj_len].date, date) < 0) {
        index ++;
        if (index >= traj_len+1) {
            /* Given date is before the earliest point in trajectory. */
            return -2;
        }
    }

    return index % traj_len;
}
