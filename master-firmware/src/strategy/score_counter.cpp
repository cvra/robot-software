#include <ch.h>
#include <hal.h>
#include <error/error.h>

#include "main.h"
#include "priorities.h"
#include "config.h"

#include "strategy/score.h"
#include "strategy/score_counter.h"
#include "strategy/state.h"
#include "protobuf/strategy.pb.h"

#define SCORE_COUNTER_STACKSIZE 1024

static THD_FUNCTION(score_counter_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    static TOPIC_DECL(score_topic, Score);

    messagebus_advertise_topic(&bus, &score_topic.topic, "/score");

    RobotState state;
    messagebus_topic_t* strategy_state_topic = messagebus_find_topic_blocking(&bus, "/state");

    NOTICE("Score initialized");
    while (true) {
        messagebus_topic_wait(strategy_state_topic, &state, sizeof(state));
        NOTICE("Received strategy state");

        Score msg;
        msg.score = 0;

        if (config_get_boolean("master/is_main_robot")) {
            msg.score += score_count_bee_on_map(state);
            msg.score += score_count_panel_on_map(state);
        }
        msg.score += score_count_bee(state);
        msg.score += score_count_switch(state);
        msg.score += score_count_tower(state);
        msg.score += score_count_tower_bonus(state);
        msg.score += score_count_balls(state);
        msg.score += score_count_wastewater_bonus(state);

        messagebus_topic_publish(&score_topic.topic, &msg, sizeof(msg));
    }
}

void score_counter_start()
{
    static THD_WORKING_AREA(score_counter_thd_wa, SCORE_COUNTER_STACKSIZE);
    chThdCreateStatic(score_counter_thd_wa, sizeof(score_counter_thd_wa),
                      SCORE_COUNTER_PRIO, score_counter_thd, NULL);
}
