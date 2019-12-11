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

    StrategyState state = initial_state();
    messagebus_topic_t* strategy_state_topic = messagebus_find_topic_blocking(&bus, "/state");

    NOTICE("Score initialized");

    while (true) {
        messagebus_topic_wait(strategy_state_topic, &state, sizeof(state));

        Score msg;

        messagebus_topic_publish(&score_topic.topic, &msg, sizeof(msg));
    }
}

void score_counter_start()
{
    static THD_WORKING_AREA(score_counter_thd_wa, SCORE_COUNTER_STACKSIZE);
    chThdCreateStatic(score_counter_thd_wa, sizeof(score_counter_thd_wa),
                      SCORE_COUNTER_PRIO, score_counter_thd, NULL);
}
