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

THD_WORKING_AREA(wa_stream, STREAM_STACKSIZE);

static msg_t stream_thread(void *p)
{
    static uint8_t buffer[64];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;

    (void) p;

    LWIP_GATEWAY(&server);

    while (1) {
        motor_driver_t *drv_list;
        uint16_t drv_list_len;
        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);

        int i;
        for (i = 0; i < drv_list_len; i++) {
            if (motor_driver_get_stream_change_status(&drv_list[i]) != 0) {
                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_CURRENT_SETPT)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "current_pid", 4);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_CURRENT));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_CURRENT_SETPT));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_MOTOR_VOLTAGE));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_VELOCITY_SETPT)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "velocity_pid", 3);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_VELOCITY));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_VELOCITY_SETPT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_POSITION_SETPT)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "position_pid", 3);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION_SETPT));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_INDEX)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "index", 2);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_INDEX));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_POSITION)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "position", 3);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_VELOCITY));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }

                if (motor_driver_get_stream_change_status(&drv_list[i])
                    & (1 << MOTOR_STREAM_MOTOR_TORQUE)) {
                    message_encode(&ctx, &mem, buffer, sizeof(buffer), "torque", 3);
                    cmp_write_str(&ctx, motor_driver_get_id(&drv_list[i]), strlen(motor_driver_get_id(&drv_list[i])));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_MOTOR_TORQUE));
                    cmp_write_float(&ctx, motor_driver_get_and_clear_stream_value(&drv_list[i], MOTOR_STREAM_POSITION));
                    message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, STREAM_PORT);
                }
            }
        }

        chThdSleepMilliseconds(STREAM_TIMESTEP_MS);
    }

    return MSG_OK;
}

void stream_init(void)
{
    chThdCreateStatic(wa_stream,
                      STREAM_STACKSIZE,
                      STREAM_PRIO,
                      stream_thread,
                      NULL);
}
