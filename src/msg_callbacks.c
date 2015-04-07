#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>

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

message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "fwd", .cb = message_fwd_callback},
    {.name = "vel", .cb = message_vel_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
