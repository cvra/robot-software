#include "msg_callbacks.h"
#include <hal.h>
#include <math.h>
#include <string.h>
#include <cmp/cmp.h>
#include <chprintf.h>
#include "usbconf.h"

#include "main.h"
#include "robot_parameters.h"
#include "motor_manager.h"
#include "unix_timestamp.h"
#include "differential_base.h"

#define TRAJ_CHUNK_BUFFER_LEN   100

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

void message_actuator_voltage_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[25];
    uint32_t actuator_id_size = 25;
    float setpoint;

    if (argc != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_voltage(&motor_manager, actuator_id, setpoint);
}

void message_actuator_torque_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[25];
    uint32_t actuator_id_size = 25;
    float setpoint;

    if (argc != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_torque(&motor_manager, actuator_id, setpoint);
}

void message_actuator_velocity_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[25];
    uint32_t actuator_id_size = 25;
    float setpoint;

    if (argc != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_velocity(&motor_manager, actuator_id, setpoint);
}

void message_actuator_position_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    char actuator_id[25];
    uint32_t actuator_id_size = 25;
    float setpoint;

    if (argc != 2) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);
    cmp_read_float(input, &setpoint);

    motor_manager_set_position(&motor_manager, actuator_id, setpoint);
}

void message_actuator_trajectory_callback(void *p, int argc, cmp_ctx_t *input)
{
    (void) p;
    (void) argc;

    unix_timestamp_t start;
    static float chunk_buffer[TRAJ_CHUNK_BUFFER_LEN][4]; // pos, vel, acc, torque
    uint32_t point_count, i, point_dimension, j;
    int32_t delta_t, start_time;
    char actuator_id[25];
    uint32_t actuator_id_size = 25;
    trajectory_chunk_t chunk;

    if (argc < 5) {
        return;
    }

    cmp_read_str(input, actuator_id, &actuator_id_size);

    cmp_read_int(input, &start.s);
    cmp_read_int(input, &start.us);
    cmp_read_int(input, &delta_t);

    cmp_read_array(input, &point_count);
    for (i = 0; i < point_count; i++) {
        cmp_read_array(input, &point_dimension);
        for (j = 0; j < point_dimension; j++) {
            cmp_read_float(input, &chunk_buffer[i][j]);
        }
    }

    start_time = timestamp_unix_to_local_us(start);
    trajectory_chunk_init(&chunk, (float*)chunk_buffer, point_count, 4, start_time, delta_t);

    motor_manager_execute_trajecory(&motor_manager, actuator_id, &chunk);
}

void wheelbase_trajectory_callback(void *p, int argc, cmp_ctx_t *input)
{
    unix_timestamp_t start;
    static float chunk_buffer[TRAJ_CHUNK_BUFFER_LEN][5];
    uint32_t point_count, i, point_dimension, j;
    int32_t dt, start_time;
    trajectory_chunk_t chunk;

    (void) p;
    (void) argc;


    cmp_read_int(input, &start.s);
    cmp_read_int(input, &start.us);
    cmp_read_int(input, &dt);


    cmp_read_array(input, &point_count);
    for (i = 0; i < point_count; ++i) {
        cmp_read_array(input, &point_dimension);
        for (j = 0; j < point_dimension; ++j) {
            cmp_read_float(input, &chunk_buffer[i][j]);
        }
    }

    start_time = timestamp_unix_to_local_us(start);

    trajectory_chunk_init(&chunk, (float *)chunk_buffer, point_count,
                                    point_dimension, start_time, dt);

    chMtxLock(&diff_base_trajectory_lock);
        int ret = trajectory_apply_chunk(&diff_base_trajectory, &chunk);
    chMtxUnlock(&diff_base_trajectory_lock);

    if (ret == 0) {
        palTogglePad(GPIOF, GPIOF_LED_READY);
    }

//    chprintf((BaseSequentialStream *)&SDU1, "retcode %d ts: %d expected ts %d\r\n", ret, dt, diff_base_trajectory.sampling_time_us);
//    chprintf((BaseSequentialStream *)&SDU1, "chunk start: %ld trajectory last: %ld\r\n", chunk.start_time_us, diff_base_trajectory.last_chunk_start_time_us);
//    chprintf((BaseSequentialStream *)&SDU1, "traj read time %d\r\n", diff_base_trajectory.read_time_us);
}




message_method_t message_callbacks[] = {
    {.name = "test", .cb = message_cb},
    {.name = "actuator_voltage", .cb = message_actuator_voltage_callback},
    {.name = "actuator_torque", .cb = message_actuator_torque_callback},
    {.name = "actuator_velocity", .cb = message_actuator_velocity_callback},
    {.name = "actuator_position", .cb = message_actuator_position_callback},
    {.name = "actuator_trajectory", .cb = message_actuator_trajectory_callback},
    {.name = "wheelbase_trajectory", .cb = wheelbase_trajectory_callback},
};

int message_callbacks_len = sizeof message_callbacks / sizeof(message_callbacks[0]);
