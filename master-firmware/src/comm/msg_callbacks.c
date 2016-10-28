#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>
#include <string.h>
#include <cmp/cmp.h>
#include <chprintf.h>
#include <error/error.h>

#include "main.h"
#include "timestamp/timestamp.h"
#include "robot_parameters.h"
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

void message_actuator_trajectory_callback(void *p, cmp_ctx_t *input)
{
    (void) p;

    unix_timestamp_t start;
    static float chunk_buffer[TRAJ_CHUNK_BUFFER_LEN][ACTUATOR_TRAJECTORY_POINT_DIMENSION];
    uint32_t point_count, i, point_dimension, j;
    int32_t delta_t, start_time;
    char actuator_id[MOTOR_ID_MAX_LEN_WITH_NUL];
    uint32_t actuator_id_size = sizeof(actuator_id);
    trajectory_chunk_t chunk;
    uint32_t array_len = 0;

    cmp_read_array(input, &array_len);
    if (array_len != 5) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);

    cmp_read_int(input, &start.s);
    cmp_read_int(input, &start.us);
    cmp_read_int(input, &delta_t);

    cmp_read_array(input, &point_count);
    if (point_count > TRAJ_CHUNK_BUFFER_LEN) {
        return;
    }
    for (i = 0; i < point_count; i++) {
        cmp_read_array(input, &point_dimension);
        if (point_dimension != ACTUATOR_TRAJECTORY_POINT_DIMENSION) {
            return;
        }
        for (j = 0; j < ACTUATOR_TRAJECTORY_POINT_DIMENSION; j++) {
            cmp_read_float(input, &chunk_buffer[i][j]);
        }
    }

    start_time = timestamp_unix_to_local_us(start);
    trajectory_chunk_init(&chunk, (float*)chunk_buffer,
                          point_count, ACTUATOR_TRAJECTORY_POINT_DIMENSION,
                          start_time, delta_t);

    DEBUG("traj chunk start: %d end: %d cur: %d",
                start_time, start_time + point_count * delta_t, timestamp_get());
    motor_manager_execute_trajecory(&motor_manager, actuator_id, &chunk);
}



struct message_method_s message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "actuator_voltage", .cb = message_actuator_voltage_callback},
    {.name = "actuator_torque", .cb = message_actuator_torque_callback},
    {.name = "actuator_velocity", .cb = message_actuator_velocity_callback},
    {.name = "actuator_position", .cb = message_actuator_position_callback},
    {.name = "actuator_trajectory", .cb = message_actuator_trajectory_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
