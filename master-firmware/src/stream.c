#include <ch.h>

#include <lwip/ip_addr.h>
#include <cmp/cmp.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <rpc_server.h>
#include <simplerpc/message.h>

#include <string.h>

#include "stream.h"
#include "motor_manager.h"
#include "main.h"
#include "priorities.h"
#include "base/base_controller.h"

#include "obstacle_avoidance/obstacle_avoidance.h"

#define STREAM_STACKSIZE 1024
#define TOPIC_NAME_LEN   40

#define POSITION_STREAM_FREQ 10
#define PATH_STREAM_FREQ     1
#define BD_STREAM_FREQ       10
#define PID_STREAM_FREQ      20

THD_WORKING_AREA(wa_stream, STREAM_STACKSIZE);

static void stream_thread(void *p)
{
    chRegSetThreadName("stream");
    static uint8_t buffer[200];
    static char topic_name[TOPIC_NAME_LEN];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;

    (void) p;

    STREAM_HOST(&server);

    // position stream
    int last_position_sent = 0;
    int last_path_sent = 0;
    int last_blocking_detect_sent = 0;
    int last_distance_pid_sent = 0;
    int last_angle_pid_sent = 0;

    while (1) {
        motor_driver_t *drv_list;
        uint16_t drv_list_len;
        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);

        int i;
        for (i = 0; i < drv_list_len; i++) {
            if (motor_driver_get_stream_change_status(&drv_list[i]) != 0) {
                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_CURRENT_SETPT)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/current_setp", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_CURRENT_SETPT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_CURRENT)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/current", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_CURRENT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_MOTOR_VOLTAGE)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/voltage", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_MOTOR_VOLTAGE));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_VELOCITY_SETPT)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/velocity_setp", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_VELOCITY_SETPT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_VELOCITY)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/velocity", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_VELOCITY));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_POSITION_SETPT)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/position_setp", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION_SETPT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_POSITION)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/position", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_INDEX)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/index", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_array(&ctx, 2);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_INDEX));
                    cmp_write_uint(&ctx, drv_list[i].stream.value_stream_index_update_count);
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_MOTOR_ENCODER)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/encoder", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_MOTOR_ENCODER));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_MOTOR_TORQUE)) {
                    strncpy(topic_name, "actuator/", TOPIC_NAME_LEN);
                    strncat(topic_name, motor_driver_get_id(&drv_list[i]), TOPIC_NAME_LEN);
                    strncat(topic_name, "/torque", TOPIC_NAME_LEN);
                    message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_MOTOR_TORQUE));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }
            }
        }

        if (last_position_sent++ * STREAM_TIMESTEP_MS >= 1000/POSITION_STREAM_FREQ) {
            last_position_sent = 0;

            float x, y, a;
            x = position_get_x_float(&robot.pos);
            y = position_get_y_float(&robot.pos);
            a = position_get_a_rad_float(&robot.pos);
            strncpy(topic_name, "position", TOPIC_NAME_LEN);
            message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
            cmp_write_array(&ctx, 3);
            cmp_write_float(&ctx, x);
            cmp_write_float(&ctx, y);
            cmp_write_float(&ctx, a);
            message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
        }

        if (last_blocking_detect_sent++ * STREAM_TIMESTEP_MS >= 1000/BD_STREAM_FREQ) {
            last_blocking_detect_sent = 0;

            bool angle_blocking = bd_get(&robot.angle_bd);
            int32_t angle_error = cs_get_error(&robot.angle_cs);
            bool distance_blocking = bd_get(&robot.distance_bd);
            int32_t distance_error = cs_get_error(&robot.distance_cs);

            strncpy(topic_name, "blocking", TOPIC_NAME_LEN);
            message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
            cmp_write_array(&ctx, 4);
            cmp_write_bool(&ctx, angle_blocking);
            cmp_write_sint(&ctx, angle_error);
            cmp_write_bool(&ctx, distance_blocking);
            cmp_write_sint(&ctx, distance_error);
            message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
        }

        if (last_distance_pid_sent++ * STREAM_TIMESTEP_MS >= 1000/PID_STREAM_FREQ) {
            last_distance_pid_sent = 0;

            int32_t dist_consign = cs_get_filtered_consign(&robot.distance_cs);
            int32_t dist_measured = cs_get_filtered_feedback(&robot.distance_cs);

            strncpy(topic_name, "distance_pid", TOPIC_NAME_LEN);
            message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
            cmp_write_array(&ctx, 2);
            cmp_write_sint(&ctx, dist_consign);
            cmp_write_sint(&ctx, dist_measured);
            message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
        }

        if (last_angle_pid_sent++ * STREAM_TIMESTEP_MS >= 1000/PID_STREAM_FREQ) {
            last_angle_pid_sent = 0;

            int32_t angle_consign = cs_get_filtered_consign(&robot.angle_cs);
            int32_t angle_measured = cs_get_filtered_feedback(&robot.angle_cs);

            strncpy(topic_name, "angle_pid", TOPIC_NAME_LEN);
            message_write_header(&ctx, &mem, buffer, sizeof(buffer), topic_name);
            cmp_write_array(&ctx, 2);
            cmp_write_sint(&ctx, angle_consign);
            cmp_write_sint(&ctx, angle_measured);
            message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
        }

        chThdSleepMilliseconds(STREAM_TIMESTEP_MS);
    }
}

void stream_init(void)
{
    chThdCreateStatic(wa_stream,
                      STREAM_STACKSIZE,
                      STREAM_PRIO,
                      stream_thread,
                      NULL);
}
