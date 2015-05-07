#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>
#include <string.h>
#include <cmp/cmp.h>

#include "robot_parameters.h"
#include "motor_control.h"
#include "unix_timestamp.h"

void message_cb(void *p, int argc, cmp_ctx_t *input)
{
    (void) argc;
    (void) p;
    bool res;
    cmp_read_bool(input, &res);

    if (res) {
        palClearPad(GPIOC, GPIOC_LED);
    } else {
        palSetPad(GPIOC, GPIOC_LED);
    }
}

void message_fwd_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    int32_t res;
    if (argc != 1) {
        return;
    }

    cmp_read_int(input, &res);
    m1_vel_setpt = (float) res / 1000.0f;
    m2_vel_setpt = - (float) res / 1000.0f;
}

void message_vel_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    int32_t fwd, rot;
    float fwd_f, rot_f;
    if (argc != 2) {
        return;
    }

    cmp_read_int(input, &fwd);
    cmp_read_int(input, &rot);
    fwd_f = (float) fwd / 1000.0f;
    rot_f = (float) rot / 1000.0f;
    m1_vel_setpt = (0.5f * ROBOT_RIGHT_WHEEL_DIRECTION / ROBOT_RIGHT_MOTOR_WHEEL_RADIUS) * (fwd_f / M_PI + ROBOT_MOTOR_WHEELBASE * rot_f);
    m2_vel_setpt = (0.5f * ROBOT_LEFT_WHEEL_DIRECTION / ROBOT_LEFT_MOTOR_WHEEL_RADIUS) * (fwd_f / M_PI - ROBOT_MOTOR_WHEELBASE * rot_f);
}

void message_traj_callback(void *p, int argc, cmp_ctx_t *input)
{
    unix_timestamp_t start;
    static float chunk_buffer[100][5];
    uint32_t point_count, i, point_dimension, j;
    int32_t dt, start_time;
    trajectory_chunk_t chunk;

    (void) p;
    (void) argc;


    cmp_read_int(input, &start.s);
    cmp_read_int(input, &start.us);
    cmp_read_int(input, &dt);


    cmp_read_array(input, &point_count);
    for (i = 0; i < point_count; ++i) {
        cmp_read_array(input, &point_dimension);
        for (j = 0; j < point_dimension; ++j) {
            cmp_read_float(input, &chunk_buffer[i][j]);
        }
    }

    start_time = timestamp_unix_to_local_us(start);

    trajectory_chunk_init(&chunk, (float *)chunk_buffer, point_count, 5, start_time, dt);

    chMtxLock(&robot_trajectory_lock);
        trajectory_apply_chunk(&robot_trajectory, &chunk);
    chMtxUnlock(&robot_trajectory_lock);
}



message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "fwd", .cb = message_fwd_callback},
    {.name = "vel", .cb = message_vel_callback},
    {.name = "traj", .cb = message_traj_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
