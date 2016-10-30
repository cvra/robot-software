#include <ch.h>

#include <msgbus/messagebus.h>
#include <error/error.h>
#include "priorities.h"
#include "base/base_controller.h"
#include "main.h"

#include "dynamic_obstacle_avoidance.h"

#define DYNAMIC_OBSTACLE_AVOIDANCE_STACKSIZE 128

typedef struct {
    float distance;
    float angle;
} beacon_signal_t;


void dynamic_obstacle_avoidance_policy(void* _robot)
{
    struct _robot* robot = (struct _robot*)_robot;

    messagebus_topic_t *beacon_topic;
    beacon_signal_t signal;

    beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    while (true) {
        messagebus_topic_wait(beacon_topic, &signal, sizeof(signal));

        if (signal.distance < 0.3) {
            trajectory_hardstop(&robot->traj);
            chThdSleepMilliseconds(50);
        }
    }
}

void dynamic_obstacle_avoidance_start(void)
{
    static THD_WORKING_AREA(dynamic_obstacle_avoidance_thd_wa,
                            DYNAMIC_OBSTACLE_AVOIDANCE_STACKSIZE);
    chThdCreateStatic(dynamic_obstacle_avoidance_thd_wa,
                      sizeof(dynamic_obstacle_avoidance_thd_wa),
                      DYNAMIC_OBSTACLE_AVOIDANCE_PRIO,
                      dynamic_obstacle_avoidance_policy,
                      &robot);
}
