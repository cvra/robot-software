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


void scara_trajectory_append_point(scara_trajectory_t *traj, const float x, const float y, const float z, const float a,
                                   scara_coordinate_t system, const float duration, const float* length)
{
    traj->frame_count += 1;
    if (traj->frame_count >= SCARA_TRAJ_MAX_NUM_FRAMES) {
        scara_panic();
    }

    if (traj->frames == NULL) {
        scara_panic();
    }

    traj->frames[traj->frame_count-1].position[0] = x;
    traj->frames[traj->frame_count-1].position[1] = y;
    traj->frames[traj->frame_count-1].position[2] = z;
    traj->frames[traj->frame_count-1].hand_angle = a;
    traj->frames[traj->frame_count-1].coordinate_type = system;

    if (traj->frame_count == 1) {
        traj->frames[0].date = scara_time_get();
    } else {
        traj->frames[traj->frame_count-1].date = traj->frames[traj->frame_count-2].date+1000000*duration;
    }

    traj->frames[traj->frame_count-1].length[0] = length[0];
    traj->frames[traj->frame_count-1].length[1] = length[1];
    traj->frames[traj->frame_count-1].length[2] = length[2];
}

void scara_trajectory_append_point_with_length(scara_trajectory_t *traj, const float x, const float y, const float z,
                                               const float a, scara_coordinate_t system,
                                               const float duration, const float l1, const float l2, const float l3)
{
    float length[3] = {l1, l2, l3};
    scara_trajectory_append_point(traj, x, y, z, a, system, duration, length);
}

void scara_trajectory_delete(scara_trajectory_t *traj)
{
    if (traj->frame_count != 0) {
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

    if (traj->frame_count == 0) {
        return 1;
    }

    if (traj->frames[last_frame].date < scara_time_get()) {
        return 1;
    }

    return 0;
}

scara_waypoint_t scara_trajectory_interpolate_waypoints(scara_waypoint_t k1, scara_waypoint_t k2, int32_t date)
{
    float t;
    scara_waypoint_t result;
    int i;

    result.date = date;

    t = (date - k1.date) / (float)(k2.date - k1.date);
    t = smoothstep(t);

    for (i=0; i<3; i++) {
        result.position[i] = interpolate(t, k1.position[i], k2.position[i]);
    }

    for (i=0; i<3; i++) {
        result.length[i] = interpolate(t, k1.length[i], k2.length[i]);
    }

    result.hand_angle = interpolate(t, k1.hand_angle, k2.hand_angle);

    return result;
}
