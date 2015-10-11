#include <string.h>
#include "rpc_callbacks.h"
#include "config.h"
#include <parameter/parameter_msgpack.h>
#include "hal.h"
#include "main.h"
#include "motor_manager.h"
#include "uavcan_node.h"

const char *error_msg_bad_format = "Error: invalid argument format.";
const char *error_msg_invalid_arg = "Error: invalid argument value.";


static bool ping_cb(void *p, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) input;
    (void) p;
    cmp_write_str(output, "pong", 4);
    return true;
}


struct param_read_err_buf_s {
    char buffer[100];
    int write_pos;
};

static void param_read_err_buffer_write_str(struct param_read_err_buf_s *buf,
                                            const char *str)
{
    int bytes_left = sizeof(buf->buffer) - buf->write_pos;
    int len = strlen(str);
    if (len > bytes_left) {
        memcpy(&buf->buffer[buf->write_pos], str, bytes_left);
        buf->write_pos += bytes_left;
    } else {
        memcpy(&buf->buffer[buf->write_pos], str, len);
        buf->write_pos += len;
    }
}

static void param_read_err_cb(void *arg, const char *id, const char *err)
{
    struct param_read_err_buf_s *buf = (struct param_read_err_buf_s *)arg;
    param_read_err_buffer_write_str(buf, "parameter ");
    if (id != NULL) {
        param_read_err_buffer_write_str(buf, id);
    }
    param_read_err_buffer_write_str(buf, ": ");
    param_read_err_buffer_write_str(buf, err);
    param_read_err_buffer_write_str(buf, "\n");
}

static bool config_update_cb(void *p, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) p;
    struct param_read_err_buf_s buf;
    buf.write_pos = 0;
    parameter_msgpack_read_cmp(&global_config, input, param_read_err_cb, (void *)&buf);
    return cmp_write_str(output, buf.buffer, buf.write_pos);
}

static bool create_motor_driver(void *p, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_len = sizeof(actuator_id);

    if (cmp_read_str(input, actuator_id, &actuator_id_len) == false) {
        cmp_write_str(output, error_msg_bad_format, strlen(error_msg_bad_format));
        return true;
    }

    motor_manager_create_driver(&motor_manager, actuator_id);
    return true;
}

static bool led_cb(void *p, cmp_ctx_t *input, cmp_ctx_t *output)
{
    (void) p;

    char led_name[32];
    uint32_t led_name_len;
    uint32_t array_len = 0;
    bool led_status;
    bool err;

    led_name_len = sizeof led_name;
    err = false;
    err = err || !cmp_read_array(input, &array_len);
    err = err || array_len != 2;
    err = err || !cmp_read_str(input, led_name, &led_name_len);
    err = err || !cmp_read_bool(input, &led_status);

    if (err == false) {
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
    return true;
}

static bool reboot_node(void *p, cmp_ctx_t *input, cmp_ctx_t *output)
{
    uint8_t id;
    (void) p;

    if(cmp_read_u8(input, &id) == false) {
        cmp_write_str(output, error_msg_bad_format,
                strlen(error_msg_bad_format));
    } else {
        uavcan_node_send_reboot(id);
    }

    return true;
}

struct service_call_method_s service_call_callbacks[] = {
    {.name="ping", .cb=ping_cb},
    {.name="config_update", .cb=config_update_cb},
    {.name="led_set", .cb=led_cb},
    {.name="actuator_create_driver", .cb=create_motor_driver},
    {.name="reboot_node", .cb=reboot_node},
};

const unsigned int service_call_callbacks_len =
    sizeof(service_call_callbacks) / sizeof(service_call_callbacks[0]);
