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
#include "protobuf/beacons.pb.h"

#define MAP_SERVER_STACKSIZE 1024

static struct {
    struct _map map;
    MUTEX_DECL(map_lock);
} map_data;

static THD_FUNCTION(map_server_thd, arg)
{
    enum strat_color_t color = *(enum strat_color_t*)arg;
    chRegSetThreadName(__FUNCTION__);

    int robot_size = config_get_integer("master/robot_size_x_mm");
    int opponent_size = config_get_integer("master/opponent_size_x_mm_default");
    map_init(&map_data.map, robot_size);

    BeaconSignal beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    AlliedPosition allied_position;

    messagebus_topic_t* allied_position_topic = messagebus_find_topic_blocking(&bus, "/allied_position");

    RobotState state;
    messagebus_topic_t* strategy_state_topic = messagebus_find_topic_blocking(&bus, "/state");

    static messagebus_watcher_t watchers[3];

    static struct {
        mutex_t lock;
        condition_variable_t cv;
        messagebus_watchgroup_t group;
    } watchgroup;

    chMtxObjectInit(&watchgroup.lock);
    chCondObjectInit(&watchgroup.cv);

    messagebus_watchgroup_init(&watchgroup.group, &watchgroup.lock, &watchgroup.cv);
    messagebus_watchgroup_watch(&watchers[0], &watchgroup.group, proximity_beacon_topic);
    messagebus_watchgroup_watch(&watchers[1], &watchgroup.group, allied_position_topic);
    messagebus_watchgroup_watch(&watchers[2], &watchgroup.group, strategy_state_topic);

    NOTICE("Map initialized");

    while (true) {
        auto map = map_server_map_lock_and_get();
        robot_size = config_get_integer("master/robot_size_x_mm");
        opponent_size = config_get_integer("master/opponent_size_x_mm_default");

        /* Create obstacle at opponent position, only consider recent beacon signal */
        if (messagebus_topic_read(proximity_beacon_topic, &beacon_signal, sizeof(beacon_signal))) {
            if (timestamp_duration_s(beacon_signal.timestamp.us, timestamp_get()) < TRAJ_MAX_TIME_DELAY_OPPONENT_DETECTION) {
                float x_opp, y_opp;
                beacon_cartesian_convert(&robot.pos,
                                         1000 * beacon_signal.range.range.distance,
                                         beacon_signal.range.angle,
                                         &x_opp, &y_opp);
                map_update_opponent_obstacle(map, x_opp, y_opp, opponent_size * 1.25, robot_size);
            } else {
                map_update_opponent_obstacle(map, 0, 0, 0, 0); // reset opponent position
            }
        }

        /* Create obstacle at allied position */
        if (messagebus_topic_read(allied_position_topic, &allied_position, sizeof(allied_position))) {
            if (timestamp_duration_s(allied_position.timestamp.us, timestamp_get()) < TRAJ_MAX_TIME_DELAY_ALLY_DETECTION) {
                map_set_ally_obstacle(map,
                                      allied_position.x,
                                      allied_position.y,
                                      robot_size,
                                      robot_size);
            } else {
                map_set_ally_obstacle(map, 0, 0, 0, 0); // reset ally position
            }
        }

        /* Update cube blocks obstacles on map depending on state */
        messagebus_topic_read(strategy_state_topic, &state, sizeof(state));
        for (int i = 0; i < MAP_NUM_BLOCKS_CUBE; i++) {
            if (state.blocks_on_map[i]) {
                const int x = BLOCK_OF_CUBES_POS[i][0];
                const int y = BLOCK_OF_CUBES_POS[i][1];
                map_set_cubes_obstacle(map, i, MIRROR_X(color, x), y, robot_size);
            } else {
                map_set_cubes_obstacle(map, i, 0, 0, 0);
            }
        }
        for (const auto& construction_zone : state.construction_zone) {
            if (construction_zone.tower_level > 0) {
                const int x = construction_zone.tower_pos[0];
                const int y = construction_zone.tower_pos[1];
                map_set_tower_obstacle(map, 0, x, y, robot_size);
            } else {
                map_set_tower_obstacle(map, 0, 0, 0, 0);
            }
        }

        map_server_map_release(map);
        messagebus_watchgroup_wait(&watchgroup.group);
    }
}

void map_server_start(enum strat_color_t color)
{
    static THD_WORKING_AREA(map_server_thd_wa, MAP_SERVER_STACKSIZE);
    chThdCreateStatic(map_server_thd_wa, sizeof(map_server_thd_wa),
                      MAP_SERVER_PRIO, map_server_thd, &color);
}

struct _map* map_server_map_lock_and_get(void)
{
    chMtxLock(&map_data.map_lock);
    return &map_data.map;
}

void map_server_map_release(struct _map* map_ptr)
{
    osalDbgAssert(map_ptr == &map_data.map, "wrong map ptr");
    chMtxUnlock(&map_data.map_lock);
}
