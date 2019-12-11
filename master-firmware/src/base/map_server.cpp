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
#include "protobuf/ally_position.pb.h"

#define MAP_SERVER_STACKSIZE 1024

static struct {
    struct _map map;
    MUTEX_DECL(map_lock);
} map_data;

static THD_FUNCTION(map_server_thd, arg)
{
    enum strat_color_t color = *(enum strat_color_t*)arg;
    (void)color;
    chRegSetThreadName(__FUNCTION__);

    int robot_size = config_get_integer("master/robot_size_x_mm");
    int opponent_size = config_get_integer("master/opponent_size_x_mm_default");
    bool enable_wall = config_get_boolean("master/is_main_robot");
    map_init(&map_data.map, robot_size, enable_wall);

    BeaconSignal beacon_signal;
    messagebus_topic_t* proximity_beacon_topic = messagebus_find_topic_blocking(&bus, "/proximity_beacon");

    AllyPosition ally_position;
    messagebus_topic_t* allied_position_topic = messagebus_find_topic_blocking(&bus, "/ally_pos");

    StrategyState state = initial_state();
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

        /* Create obstacle at ally position */
        if (messagebus_topic_read(allied_position_topic, &ally_position, sizeof(ally_position))) {
            map_set_ally_obstacle(map,
                                  ally_position.x,
                                  ally_position.y,
                                  robot_size,
                                  robot_size);
        } else {
            map_set_ally_obstacle(map, 0, 0, 0, 0); // reset ally position
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

void map_server_enable_opponent(struct _map* map_ptr, bool enable)
{
    map_ptr->enable_opponent = enable;
}
