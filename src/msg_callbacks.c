#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>
#include <string.h>

#include "robot_parameters.h"
#include "motor_control.h"

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

static void read_traj(trajectory_frame_t *traj, int traj_len, cmp_ctx_t *input)
{
    uint32_t point_count, tuple_size, i;
    static trajectory_frame_t newtraj[ROBOT_TRAJ_LEN];

    memset(newtraj, 0, sizeof newtraj);

    cmp_read_array(input, &point_count);

    for (i = 0; i < point_count; ++i) {
        cmp_read_array(input, &tuple_size);
        cmp_read_int(input, &newtraj[i].date.s);
        cmp_read_int(input, &newtraj[i].date.us);
        cmp_read_float(input, &newtraj[i].val);
    }

    trajectory_merge(traj, traj_len, newtraj, ROBOT_TRAJ_LEN);
}

void message_traj_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;

    if (argc != 5) {
        return;
    }

    chMtxLock(&robot_traj.lock);
        read_traj(robot_traj.x, ROBOT_TRAJ_LEN, input);
        read_traj(robot_traj.y, ROBOT_TRAJ_LEN, input);
        read_traj(robot_traj.theta, ROBOT_TRAJ_LEN, input);
        read_traj(robot_traj.speed, ROBOT_TRAJ_LEN, input);
        read_traj(robot_traj.omega, ROBOT_TRAJ_LEN, input);
    chMtxUnlock(&robot_traj.lock);
}

message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "fwd", .cb = message_fwd_callback},
    {.name = "vel", .cb = message_vel_callback},
    {.name = "traj", .cb = message_traj_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
