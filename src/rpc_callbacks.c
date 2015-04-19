#include <string.h>
#include "rpc_callbacks.h"
#include "config.h"
#include <parameter/parameter_msgpack.h>

static void ping_cb(void *p, int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) input;
    (void) p;
    cmp_write_str(output, "pong", 4);
}

static void msgpack_error_cb(void *arg, const char *id, const char *err)
{
    cmp_ctx_t *output = (cmp_ctx_t *)arg;
    cmp_write_str(output, err, strlen(err));
}

static void config_update_cb(void *p, int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) output;
    (void) p;
    parameter_msgpack_read_cmp(&global_config, input, msgpack_error_cb, (void *)output);
}

service_call_method service_call_callbacks[] = {
    {.name="ping", .cb=ping_cb},
    {.name="config_update", .cb=config_update_cb}
};

const unsigned int service_call_callbacks_len =
    sizeof(service_call_callbacks) / sizeof(service_call_callbacks[0]);
