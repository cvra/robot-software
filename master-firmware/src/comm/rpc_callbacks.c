#include <string.h>
#include "rpc_callbacks.h"
#include "config.h"
#include <parameter/parameter_msgpack.h>
#include "hal.h"
#include "main.h"
#include "motor_manager.h"
#include "uavcan_node.h"

const char* error_msg_bad_format = "Error: invalid argument format.";

static bool ping_cb(void* p, cmp_ctx_t* input, cmp_ctx_t* output)
{
    (void)input;
    (void)p;
    cmp_write_str(output, "pong", 4);
    return true;
}

struct param_read_err_buf_s {
    char buffer[100];
    int write_pos;
};

static void param_read_err_buffer_write_str(struct param_read_err_buf_s* buf,
                                            const char* str)
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

static void param_read_err_cb(void* arg, const char* id, const char* err)
{
    struct param_read_err_buf_s* buf = (struct param_read_err_buf_s*)arg;
    param_read_err_buffer_write_str(buf, "parameter ");
    if (id != NULL) {
        param_read_err_buffer_write_str(buf, id);
    }
    param_read_err_buffer_write_str(buf, ": ");
    param_read_err_buffer_write_str(buf, err);
    param_read_err_buffer_write_str(buf, "\n");
}

static bool config_update_cb(void* p, cmp_ctx_t* input, cmp_ctx_t* output)
{
    (void)p;
    struct param_read_err_buf_s buf;
    buf.write_pos = 0;
    parameter_msgpack_read_cmp(&global_config, input, param_read_err_cb, (void*)&buf);
    if (buf.write_pos == 0) {
        return true;
    } else {
        return cmp_write_str(output, buf.buffer, buf.write_pos);
    }
}

static bool create_motor_driver(void* p, cmp_ctx_t* input, cmp_ctx_t* output)
{
    (void)p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_len = sizeof(actuator_id);

    if (cmp_read_str(input, actuator_id, &actuator_id_len) == false) {
        cmp_write_str(output, error_msg_bad_format, strlen(error_msg_bad_format));
        return true;
    }

    motor_manager_create_driver(&motor_manager, actuator_id);
    return true;
}

struct service_call_method_s service_call_callbacks[] = {
    {.name = "ping", .cb = ping_cb},
    {.name = "config_update", .cb = config_update_cb},
    {.name = "actuator_create_driver", .cb = create_motor_driver},
};

const unsigned int service_call_callbacks_len =
    sizeof(service_call_callbacks) / sizeof(service_call_callbacks[0]);
