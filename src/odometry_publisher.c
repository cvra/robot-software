#include <ch.h>

#include <lwip/ip_addr.h>
#include <cmp/cmp.h>
#include <cmp_mem_access/cmp_mem_access.h>
#include <rpc_server.h>
#include <simplerpc/message.h>

#include "priorities.h"
#include "robot_pose.h"
#include "odometry_publisher.h"

#define ODOMETRY_PUBLISHER_STACKSIZE 1024

THD_WORKING_AREA(wa_odometry_publisher, ODOMETRY_PUBLISHER_STACKSIZE);

static void odometry_publisher_thread(void *p)
{
    static uint8_t buffer[64];
    cmp_ctx_t ctx;
    cmp_mem_access_t mem;
    ip_addr_t server;

    (void) p;

    chRegSetThreadName("odometry_publisher");

    ODOMETRY_PUBLISHER_HOST(&server);

    while (1) {
            message_write_header(&ctx, &mem, buffer, sizeof buffer,
                                 "odometry_raw");
            chMtxLock(&robot_pose_lock);
                //odometry_base_get_pose(&robot_base, &robot_pose);
                cmp_write_array(&ctx, 3);
                cmp_write_float(&ctx, robot_pose.x);
                cmp_write_float(&ctx, robot_pose.y);
                cmp_write_float(&ctx, robot_pose.theta);
            chMtxUnlock(&robot_pose_lock);

            message_transmit(buffer, cmp_mem_access_get_pos(&mem),
                             &server, ODOMETRY_PUBLISHER_PORT);

            chThdSleepMilliseconds(ODOMETRY_PUBLISHER_TIMESTEP_MS);
    }
}

void odometry_publisher_init(void)
{
    chThdCreateStatic(wa_odometry_publisher,
                       ODOMETRY_PUBLISHER_STACKSIZE,
                       ODOMETRY_PUBLISHER_PRIO,
                       odometry_publisher_thread, NULL);
}
