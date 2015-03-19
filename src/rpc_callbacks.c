#include "rpc_callbacks.h"

static void ping_cb(int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) input;
    cmp_write_str(output, "pong", 4);
}

service_call_method service_call_callbacks[] = {
    {.name="ping", .cb=ping_cb}
};

const unsigned int service_call_callbacks_len =
    sizeof(service_call_callbacks) / sizeof(service_call_callbacks[0]);
