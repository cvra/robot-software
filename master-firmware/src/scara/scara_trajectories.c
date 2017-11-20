#include <stdlib.h>
#include <string.h>

#include "scara_port.h"
#include "scara_trajectories.h"

int32_t (*debra_scara_time_get)(void);


static float smoothstep(float t)
{
    if(t < 0.0f) return 0.0f;
    if(t > 1.0f) return 1.0f;
    return t*t*t*(t*(6.0f*t-15.0f)+10.0f);
}

static float interpolate(float t, float a, float b)
{
    return (1 - t) * a + t * b;
}

void scara_trajectory_init(scara_trajectory_t *traj) {
    memset(traj, 0, sizeof(scara_trajectory_t));
}


void scara_trajectory_append_point(scara_trajectory_t *traj, position_3d_t pos,
                                   scara_coordinate_t system, float duration, const float* length)
{
    traj->frame_count += 1;
    if (traj->frame_count >= SCARA_TRAJ_MAX_NUM_FRAMES) {
        scara_panic();
    }

    if (traj->frames == NULL) {
        scara_panic();
    }

    traj->frames[traj->frame_count-1].position.x = pos.x;
    traj->frames[traj->frame_count-1].position.y = pos.y;
    traj->frames[traj->frame_count-1].position.z = pos.z;
    traj->frames[traj->frame_count-1].coordinate_type = system;

    if (traj->frame_count == 1) {
        traj->frames[0].date = scara_time_get();
    } else {
        traj->frames[traj->frame_count-1].date = traj->frames[traj->frame_count-2].date+1000000*duration;
    }

    traj->frames[traj->frame_count-1].length[0] = length[0];
    traj->frames[traj->frame_count-1].length[1] = length[1];
}

void scara_trajectory_delete(scara_trajectory_t *traj)
{
    if (!scara_trajectory_is_empty(traj)) {
        traj->frame_count = 0;
    }
}

void scara_trajectory_copy(scara_trajectory_t *dest, scara_trajectory_t *src)
{
    dest->frame_count = src->frame_count;
    memmove(dest->frames, src->frames, dest->frame_count * sizeof(scara_waypoint_t));
}

int scara_trajectory_finished(scara_trajectory_t *traj)
{
    int last_frame = traj->frame_count - 1;

    if (scara_trajectory_is_empty(traj)) {
        return 1;
    }

    if (traj->frames[last_frame].date < scara_time_get()) {
        return 1;
    }

    return 0;
}

bool scara_trajectory_is_empty(scara_trajectory_t* trajectory)
{
    return trajectory->frame_count == 0;
}

scara_waypoint_t scara_trajectory_interpolate_waypoints(scara_waypoint_t k1, scara_waypoint_t k2, int32_t date)
{
    float t;
    scara_waypoint_t result = k2;
    int i;

    result.date = date;

    t = (date - k1.date) / (float)(k2.date - k1.date);
    t = smoothstep(t);

    result.position.x = interpolate(t, k1.position.x, k2.position.x);
    result.position.y = interpolate(t, k1.position.y, k2.position.y);
    result.position.z = interpolate(t, k1.position.z, k2.position.z);

    for (i=0; i<2; i++) {
        result.length[i] = interpolate(t, k1.length[i], k2.length[i]);
    }

    return result;
}
