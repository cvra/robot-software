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
#include "strategy/state.h"

#define MAP_SERVER_STACKSIZE 1024

static struct _map map;

static THD_FUNCTION(map_server_thd, arg)
{
    enum strat_color_t color = *(enum strat_color_t*)arg;
    chRegSetThreadName(__FUNCTION__);

    int robot_size = config_get_integer("master/robot_size_x_mm");
    int opponent_size = config_get_integer("master/opponent_size_x_mm_default");
    map_init(&map, robot_size);

    beacon_signal_t beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    RobotState state;
    messagebus_topic_t* strategy_state_topic = messagebus_find_topic_blocking(&bus, "/state");

    NOTICE("Map initialized");
    while (true) {
        robot_size = config_get_integer("master/robot_size_x_mm");
        opponent_size = config_get_integer("master/opponent_size_x_mm_default");

        /* Create obstacle at opponent position, only consider recent beacon signal */
        messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal));
        if (timestamp_duration_s(beacon_signal.timestamp, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
            float x_opp, y_opp;
            beacon_cartesian_convert(&robot.pos, 1000 * beacon_signal.distance, beacon_signal.heading, &x_opp, &y_opp);
            map_update_opponent_obstacle(&map, x_opp, y_opp, opponent_size * 1.25, robot_size);
        } else {
            map_update_opponent_obstacle(&map, 0, 0, 0, 0); // reset opponent position
        }

        /* Update cube blocks obstacles on map depending on state */
        messagebus_topic_read(strategy_state_topic, &state, sizeof(state));
        for (int i = 0; i < MAP_NUM_BLOCKS_CUBE; i++) {
            if (state.blocks_on_map[i]) {
                const int x = state.blocks_pos[i][0];
                const int y = state.blocks_pos[i][1];
                map_set_cubes_obstacle(&map, i, MIRROR_X(color, x), y, robot_size);
            } else {
                map_set_cubes_obstacle(&map, i, 0, 0, 0);
            }
        }
        if (state.tower_level > 0) {
            map_set_tower_obstacle(&map, 0, state.tower_pos[0], state.tower_pos[1], robot_size);
        } else {
            map_set_tower_obstacle(&map, 0, 0, 0, 0);
        }

        chThdSleepMilliseconds(1000 / MAP_SERVER_FREQUENCY);
    }
}

void map_server_start(enum strat_color_t color)
{
    static THD_WORKING_AREA(map_server_thd_wa, MAP_SERVER_STACKSIZE);
    chThdCreateStatic(map_server_thd_wa, sizeof(map_server_thd_wa),
                      MAP_SERVER_PRIO, map_server_thd, &color);
}
