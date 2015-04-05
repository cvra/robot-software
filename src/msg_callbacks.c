#include "msg_callbacks.h"
#include <hal.h>

#include "motor_control.h"

void message_cb(int argc, cmp_ctx_t *input)
{
    (void) argc;
    bool res;
    cmp_read_bool(input, &res);

    if (res) {
        palClearPad(GPIOC, GPIOC_LED);
    } else {
        palSetPad(GPIOC, GPIOC_LED);
    }
}

void message_fwd_callback(int argc, cmp_ctx_t *input)
{
    int res;
    if (argc != 1) {
        return;
    }

    cmp_read_int(input, &res);
    m1_vel_setpt = res;
    m2_vel_setpt = -res;
}

message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "fwd", .cb = message_fwd_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
