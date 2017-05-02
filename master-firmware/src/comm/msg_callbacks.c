#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>
#include <string.h>
#include <cmp/cmp.h>
#include <chprintf.h>
#include <error/error.h>

#include "main.h"
#include "timestamp/timestamp.h"
#include "motor_manager.h"
#include "unix_timestamp.h"

#define TRAJ_CHUNK_BUFFER_LEN   100

void message_cb(void *p, cmp_ctx_t *input)
{
    (void) p;
    bool res;
    cmp_read_bool(input, &res);

    if (res) {
        board_led_on();
    } else {
        board_led_off();
    }
}

void message_actuator_voltage_callback(void *p, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_size = sizeof(actuator_id);
    float setpoint;
    uint32_t array_len = 0;

    cmp_read_array(input, &array_len);
    if (array_len != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_voltage(&motor_manager, actuator_id, setpoint);
}

void message_actuator_torque_callback(void *p, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_size = sizeof(actuator_id);
    float setpoint;
    uint32_t array_len = 0;

    cmp_read_array(input, &array_len);
    if (array_len != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_torque(&motor_manager, actuator_id, setpoint);
}

void message_actuator_velocity_callback(void *p, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_size = sizeof(actuator_id);
    float setpoint;
    uint32_t array_len = 0;

    cmp_read_array(input, &array_len);
    if (array_len != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_velocity(&motor_manager, actuator_id, setpoint);
}

void message_actuator_position_callback(void *p, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_size = sizeof(actuator_id);
    float setpoint;
    uint32_t array_len = 0;

    cmp_read_array(input, &array_len);
    if (array_len != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_position(&motor_manager, actuator_id, setpoint);
}

struct message_method_s message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "actuator_voltage", .cb = message_actuator_voltage_callback},
    {.name = "actuator_torque", .cb = message_actuator_torque_callback},
    {.name = "actuator_velocity", .cb = message_actuator_velocity_callback},
    {.name = "actuator_position", .cb = message_actuator_position_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
