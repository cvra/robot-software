#include <ch.h>
#include <hal.h>
#include <utility>
#include <unordered_map>
#include "main.h"
#include "state_estimation.hpp"
#include "ranging_thread.h"
#include "state_estimation_thread.h"
#include "imu_thread.h"

#define CACHE_ENTRIES 5

static cache_t anchor_positions_cache;
static cache_entry_t anchor_positions_cache_entries[CACHE_ENTRIES];
static anchor_position_msg_t anchor_positions_cache_entries_content[CACHE_ENTRIES];
static MUTEX_DECL(anchor_positions_cache_lock);

static THD_WORKING_AREA(state_estimation_wa, 1024);
static THD_FUNCTION(state_estimation_thd, arg)
{
    (void) arg;

    messagebus_topic_t *range_topic, *anchor_pos_topic, *imu_topic;
    struct {
        mutex_t lock;
        condition_variable_t cv;
        messagebus_watchgroup_t group;
        messagebus_watcher_t watchers[3];
    } watchgroup;

    messagebus_topic_t state_estimation_topic;
    MUTEX_DECL(state_estimation_topic_lock);
    CONDVAR_DECL(state_estimation_topic_condvar);
    position_estimation_msg_t state_estimation_topic_content;

    messagebus_topic_init(&state_estimation_topic,
                          &state_estimation_topic_lock,
                          &state_estimation_topic_condvar,
                          &state_estimation_topic_content,
                          sizeof(state_estimation_topic_content));
    messagebus_advertise_topic(&bus, &state_estimation_topic, "/ekf/state");

    struct {
        parameter_namespace_t ns;
        parameter_t process_variance;
        parameter_t range_variance;
    } params;

    parameter_namespace_declare(&params.ns, &parameter_root, "ekf");
    parameter_scalar_declare_with_default(&params.process_variance,
                                          &params.ns,
                                          "process_variance",
                                          4e-6);
    parameter_scalar_declare_with_default(&params.range_variance, &params.ns, "range_variance",
                                          9e-4);

    cache_init(&anchor_positions_cache, anchor_positions_cache_entries, CACHE_ENTRIES);

    for (int i = 0; i < CACHE_ENTRIES; i++) {
        anchor_positions_cache_entries[i].payload = (void *)&anchor_positions_cache_entries_content[i];
    }

    // TODO configure variance
    RadioPositionEstimator estimator;

    anchor_pos_topic = messagebus_find_topic_blocking(&bus, "/anchors_pos");
    range_topic = messagebus_find_topic_blocking(&bus, "/range");
    imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    /* Prepare to listen on all groups. */
    chMtxObjectInit(&watchgroup.lock);
    chCondObjectInit(&watchgroup.cv);
    messagebus_watchgroup_init(&watchgroup.group, &watchgroup.lock, &watchgroup.cv);
    messagebus_watchgroup_watch(&watchgroup.watchers[0],
                                &watchgroup.group,
                                anchor_pos_topic);
    messagebus_watchgroup_watch(&watchgroup.watchers[1],
                                &watchgroup.group,
                                range_topic);
    messagebus_watchgroup_watch(&watchgroup.watchers[2],
                                &watchgroup.group,
                                imu_topic);

    while (true) { messagebus_topic_t *topic;
        topic = messagebus_watchgroup_wait(&watchgroup.group);

        if (topic == anchor_pos_topic) {
            // If we got a new beacon position, store it in the map
            anchor_position_msg_t msg;
            messagebus_topic_read(topic, &msg, sizeof(msg));

            state_estimation_anchor_cache_acquire();

            cache_entry_t *entry = cache_entry_get(&anchor_positions_cache, msg.anchor_addr);
            anchor_position_msg_t *dst;
            if (entry == NULL) {
                entry = cache_entry_allocate(&anchor_positions_cache, msg.anchor_addr);
            }
            dst = (anchor_position_msg_t *)entry->payload;
            memcpy(dst, &msg, sizeof(anchor_position_msg_t));

            state_estimation_anchor_cache_release();
        } else if (topic == range_topic) {
            // If we got a range, feed it to the estimator
            range_msg_t msg;
            messagebus_topic_read(topic, &msg, sizeof(msg));

            estimator.measurementVariance = parameter_scalar_read(&params.range_variance);

            state_estimation_anchor_cache_acquire();
            if (cache_entry_get(&anchor_positions_cache, msg.anchor_addr)) {
                palTogglePad(GPIOB, GPIOB_LED_ERROR);
            }
            state_estimation_anchor_cache_release();

        } else if (topic == imu_topic) {
            // TODO: Better source of periodic interrupts than IMU?
            imu_msg_t imu_msg;
            messagebus_topic_read(topic, &imu_msg, sizeof(imu_msg));

            estimator.processVariance = parameter_scalar_read(&params.process_variance);
            estimator.predict();
            position_estimation_msg_t pos_msg;
            pos_msg.timestamp = imu_msg.timestamp;
            std::tie(pos_msg.x, pos_msg.y) = estimator.getPosition();
            messagebus_topic_publish(&state_estimation_topic, &pos_msg, sizeof(pos_msg));
        }
    }
}

void state_estimation_start(void)
{
    chThdCreateStatic(state_estimation_wa,
                      sizeof(state_estimation_wa),
                      NORMALPRIO,
                      state_estimation_thd,
                      NULL);
}

cache_t *state_estimation_anchor_cache_acquire(void)
{
    chMtxLock(&anchor_positions_cache_lock);
    return &anchor_positions_cache;
}

void state_estimation_anchor_cache_release(void)
{
    chMtxUnlock(&anchor_positions_cache_lock);
}

