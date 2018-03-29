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
    enum strat_color_t color = *(enum strat_color_t*)arg;
    chRegSetThreadName(__FUNCTION__);

    const int robot_size = config_get_integer("master/robot_size_x_mm");
    const int opponent_size = config_get_integer("master/opponent_size_x_mm_default");
    map_init(robot_size);

    beacon_signal_t beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    NOTICE("Map initialized");
    while (true) {
        /* Create obstacle at opponent position, only consider recent beacon signal */
        messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));
        if (timestamp_duration_s(beacon_signal.timestamp, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
            float x_opp, y_opp;
            beacon_cartesian_convert(&robot.pos, 1000 * beacon_signal.distance, beacon_signal.heading, &x_opp, &y_opp);
            map_update_opponent_obstacle(x_opp, y_opp, opponent_size * 1.25, robot_size);
        } else {
            map_update_opponent_obstacle(0, 0, 0, 0); // reset opponent position
        }

        map_set_cubes_obstacle(map_get_cubes_obstacle(0), MIRROR_X(color,  850),  540, robot_size);
        map_set_cubes_obstacle(map_get_cubes_obstacle(1), MIRROR_X(color,  300), 1190, robot_size);
        map_set_cubes_obstacle(map_get_cubes_obstacle(2), MIRROR_X(color, 1100), 1500, robot_size);
        map_set_cubes_obstacle(map_get_cubes_obstacle(3), MIRROR_X(color, 1900), 1500, robot_size);
        map_set_cubes_obstacle(map_get_cubes_obstacle(4), MIRROR_X(color, 2700), 1190, robot_size);
        map_set_cubes_obstacle(map_get_cubes_obstacle(5), MIRROR_X(color, 2150),  540, robot_size);

        chThdSleepMilliseconds(1000 / MAP_SERVER_FREQUENCY);
    }
}

void map_server_start(enum strat_color_t color)
{
    static THD_WORKING_AREA(map_server_thd_wa, MAP_SERVER_STACKSIZE);
    chThdCreateStatic(map_server_thd_wa, sizeof(map_server_thd_wa),
                      MAP_SERVER_PRIO, map_server_thd, &color);
}
