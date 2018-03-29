#include <ch.h>
#include <hal.h>
#include <string.h>
#include <error/error.h>
#include <timestamp/timestamp.h>

#include "config.h"
#include "main.h"
#include "priorities.h"

#include "base/base_controller.h"
#include "base/map.h"
#include "base/map_server.h"
#include "robot_helpers/beacon_helpers.h"
#include "robot_helpers/trajectory_helpers.h"

#define MAP_SERVER_STACKSIZE 512

static THD_FUNCTION(map_server_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    map_init(config_get_integer("master/robot_size_x_mm"));

    beacon_signal_t beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    NOTICE("Map initialized");
    while (true) {
        /* Create obstacle at opponent position, only consider recent beacon signal */
        messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));
        if (timestamp_duration_s(beacon_signal.timestamp, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
            float x_opp, y_opp;
            beacon_cartesian_convert(&robot.pos, 1000 * beacon_signal.distance, beacon_signal.heading, &x_opp, &y_opp);
            map_update_opponent_obstacle(x_opp, y_opp, robot.opponent_size * 1.25, robot.robot_size);
        } else {
            map_update_opponent_obstacle(0, 0, 0, 0); // reset opponent position
        }

        chThdSleepMilliseconds(1000 / MAP_SERVER_FREQUENCY);
    }
}

void map_server_start()
{
    static THD_WORKING_AREA(map_server_thd_wa, MAP_SERVER_STACKSIZE);
    chThdCreateStatic(map_server_thd_wa, sizeof(map_server_thd_wa),
                      MAP_SERVER_PRIO, map_server_thd, NULL);
}
