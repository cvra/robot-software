#include <string.h>
#include "rpc_callbacks.h"
#include "config.h"
#include <parameter/parameter_msgpack.h>
#include "hal.h"

const char *error_msg_bad_argc = "Error: invalid argument count.";
const char *error_msg_bad_format = "Error: invalid argument format.";
const char *error_msg_invalid_arg = "Error: invalid argument value.";


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
    cmp_write_array(output, 2);
    cmp_write_str(output, id, strlen(id));
    cmp_write_str(output, err, strlen(err));
}

static void config_update_cb(void *p, int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) argc;
    (void) output;
    (void) p;
    parameter_msgpack_read_cmp(&global_config, input, msgpack_error_cb, (void *)output);
}

static void led_cb(void *p, int argc, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) output;
    (void) p;

    char led_name[32];
    uint32_t led_name_len;
    bool led_status;
    bool success;

    if (argc != 2) {
        cmp_write_str(output, error_msg_bad_argc, strlen(error_msg_bad_argc));
    }

    led_name_len = sizeof led_name;
    success = true;
    success &= cmp_read_str(input, led_name, &led_name_len);
    success &= cmp_read_bool(input, &led_status);

    if (success) {
        if (!strcmp(led_name, "ready")) {
            palWritePad(GPIOF, GPIOF_LED_READY, led_status);
        } else if (!strcmp(led_name, "debug")) {
            palWritePad(GPIOF, GPIOF_LED_DEBUG, led_status);
        } else if (!strcmp(led_name, "error")) {
            palWritePad(GPIOF, GPIOF_LED_ERROR, led_status);
        } else if (!strcmp(led_name, "power_error")) {
            palWritePad(GPIOF, GPIOF_LED_POWER_ERROR, led_status);
        } else if (!strcmp(led_name, "pc_error")) {
            palWritePad(GPIOF, GPIOF_LED_PC_ERROR, led_status);
        } else if (!strcmp(led_name, "bus_error")) {
            palWritePad(GPIOF, GPIOF_LED_BUS_ERROR, led_status);
        } else if (!strcmp(led_name, "yellow_1")) {
            palWritePad(GPIOF, GPIOF_LED_YELLOW_1, led_status);
        } else if (!strcmp(led_name, "yellow_2")) {
            palWritePad(GPIOF, GPIOF_LED_YELLOW_2, led_status);
        } else if (!strcmp(led_name, "green_1")) {
            palWritePad(GPIOF, GPIOF_LED_GREEN_1, led_status);
        } else if (!strcmp(led_name, "green_2")) {
            palWritePad(GPIOF, GPIOF_LED_GREEN_2, led_status);
        } else {
            cmp_write_str(output, error_msg_invalid_arg,
                                  strlen(error_msg_invalid_arg));

        }
    } else {
        cmp_write_str(output, error_msg_bad_format,
                              strlen(error_msg_bad_format));
    }
}

service_call_method service_call_callbacks[] = {
    {.name="ping", .cb=ping_cb},
    {.name="config_update", .cb=config_update_cb},
    {.name="led_set", .cb=led_cb},
};

const unsigned int service_call_callbacks_len =
    sizeof(service_call_callbacks) / sizeof(service_call_callbacks[0]);
