#include <ch.h>
#include <hal.h>
#include "main.h"
#include "state_estimation.hpp"
#include "ranging_thread.h"
#include "state_estimation_thread.h"
#include "anchor_position_cache.h"
#include "imu_thread.h"

/** Parameters for this service. */
static struct {
    parameter_namespace_t ns;
    parameter_t process_variance;
    parameter_t range_variance;
} params;

/** Creates the parameters. */
static void parameters_init(parameter_namespace_t* parent);

static THD_WORKING_AREA(state_estimation_wa, 1024);
static THD_FUNCTION(state_estimation_thd, arg)
{
    (void)arg;

    messagebus_topic_t *range_topic, *imu_topic;
    struct {
        mutex_t lock;
        condition_variable_t cv;
        messagebus_watchgroup_t group;
        messagebus_watcher_t watchers[2];
    } watchgroup;

    messagebus_topic_t state_estimation_topic;
    MUTEX_DECL(state_estimation_topic_lock);
    CONDVAR_DECL(state_estimation_topic_condvar);
    position_estimation_msg_t state_estimation_topic_content;

    messagebus_topic_init(&state_estimation_topic,
                          &state_estimation_topic_lock,
                          &state_estimation_topic_condvar, &state_estimation_topic_content,
                          sizeof(state_estimation_topic_content));
    messagebus_advertise_topic(&bus, &state_estimation_topic, "/ekf/state");

    parameters_init(&parameter_root);

    // TODO configure variance
    RadioPositionEstimator estimator;

    range_topic = messagebus_find_topic_blocking(&bus, "/range");
    imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    /* Prepare to listen on all groups. */
    chMtxObjectInit(&watchgroup.lock);
    chCondObjectInit(&watchgroup.cv);
    messagebus_watchgroup_init(&watchgroup.group, &watchgroup.lock, &watchgroup.cv);
    messagebus_watchgroup_watch(&watchgroup.watchers[0],
                                &watchgroup.group,
                                range_topic);
    messagebus_watchgroup_watch(&watchgroup.watchers[1],
                                &watchgroup.group,
                                imu_topic);

    while (true) {
        messagebus_topic_t* topic;
        topic = messagebus_watchgroup_wait(&watchgroup.group);

        if (topic == range_topic) {
            // If we got a range, feed it to the estimator
            range_msg_t msg;
            anchor_position_msg_t* anchor_pos;
            messagebus_topic_read(topic, &msg, sizeof(msg));

            // Discard messages with range greater than 1km, as they are
            // probably the result of an underflow if the tag is too close to
            // the anchor.
            if (msg.range > 1000) {
                continue;
            }

            estimator.measurementVariance = parameter_scalar_read(&params.range_variance);

            anchor_pos = anchor_position_cache_get(msg.anchor_addr);

            if (anchor_pos) {
                float pos[3] = {anchor_pos->x, anchor_pos->y, anchor_pos->z};
                estimator.processDistanceMeasurement(pos, msg.range);
            }

        } else if (topic == imu_topic) {
            // TODO: Better source of periodic interrupts than IMU?
            imu_msg_t imu_msg;
            messagebus_topic_read(topic, &imu_msg, sizeof(imu_msg));

            estimator.processVariance = parameter_scalar_read(&params.process_variance);
            estimator.predict();
            position_estimation_msg_t pos_msg;
            pos_msg.timestamp = imu_msg.timestamp;
            pos_msg.x = estimator.state(0);
            pos_msg.y = estimator.state(1);
            pos_msg.z = estimator.state(2);
            pos_msg.variance_x = estimator.covariance(0, 0);
            pos_msg.variance_y = estimator.covariance(1, 1);
            pos_msg.variance_z = estimator.covariance(2, 2);
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

static void parameters_init(parameter_namespace_t* parent)
{
    parameter_namespace_declare(&params.ns, parent, "ekf");
    parameter_scalar_declare_with_default(&params.process_variance,
                                          &params.ns,
                                          "process_variance",
                                          4e-6);
    parameter_scalar_declare_with_default(&params.range_variance, &params.ns, "range_variance",
                                          9e-4);
}
