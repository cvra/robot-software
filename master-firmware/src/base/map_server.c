#include <ch.h>
#include <hal.h>

#include <string.h>

#include <error/error.h>

#include "config.h"
#include "map.h"
#include "map_server.h"
#include "priorities.h"

#define MAP_SERVER_STACKSIZE 512

static THD_FUNCTION(map_server_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    map_init(config_get_integer("master/robot_size_x_mm"));

    NOTICE("Map initialized");
    while (true) {
        chThdSleepMilliseconds(1000 / MAP_SERVER_FREQUENCY);
    }
}

void map_server_start()
{
    static THD_WORKING_AREA(map_server_thd_wa, MAP_SERVER_STACKSIZE);
    chThdCreateStatic(map_server_thd_wa, sizeof(map_server_thd_wa),
                      MAP_SERVER_PRIO, map_server_thd, NULL);
}
