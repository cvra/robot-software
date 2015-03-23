#include "msg_callbacks.h"
#include <hal.h>

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

message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb}
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
