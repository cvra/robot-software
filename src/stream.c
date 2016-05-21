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

#define STREAM_STACKSIZE 1024
#define TOPIC_NAME_LEN   40

THD_WORKING_AREA(wa_stream, STREAM_STACKSIZE);

static void stream_thread(void *p)
{
    static uint8_t buffer[64];
    static char topic_name[TOPIC_NAME_LEN];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;

    (void) p;

    STREAM_HOST(&server);


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
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_INDEX));
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
