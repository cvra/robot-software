#include <ch.h>
#include <hal.h>
#include "main.h"
#include "uavcan/topics_publisher.hpp"
#include "imu_thread.h"

#include <uavcan/equipment/ahrs/RawIMU.hpp>

#define WATCHED_TOPIC_CNT 1

uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::ahrs::RawIMU> > raw_imu_pub;

static BSEMAPHORE_DECL(imu_topic_signaled, true);

static parameter_t publish_imu;
static parameter_namespace_t publish_sensor_ns;


static void topics_watcher(void *p)
{
    (void) p;

    chRegSetThreadName(__FUNCTION__);

    static messagebus_watcher_t watchers[WATCHED_TOPIC_CNT];

    static struct {
        mutex_t lock;
        condition_variable_t cv;
        messagebus_watchgroup_t group;
    } watchgroup;

    static messagebus_topic_t *imu_topic;


    /* Create a watchgroup. */
    chMtxObjectInit(&watchgroup.lock);
    chCondObjectInit(&watchgroup.cv);
    messagebus_watchgroup_init(&watchgroup.group, &watchgroup.cv, &watchgroup.lock);

    /* Adds all the topics to the watchers. */
    /* TODO: If a topic is missing, the thread will hang forever.
     * Maybe it would be better to check in the loop if the topic exists. */
    imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    messagebus_watchgroup_watch(&watchers[0],
                                &watchgroup.group,
                                imu_topic);

    while (1) {
        /* Wait for a topic publication. */
        messagebus_topic_t *topic;
        topic = messagebus_watchgroup_wait(&watchgroup.group);

        if (topic == imu_topic && parameter_boolean_get(&publish_imu)) {
            chBSemSignal(&imu_topic_signaled);
        } else if (topic == attitude_topic && parameter_boolean_get(&publish_imu)) {
            chBSemSignal(&attitude_topic_signaled);
        }
    }
}

void topics_publisher_start(Node &node)
{
    parameter_namespace_declare(&publish_sensor_ns, &parameter_root, "publish_sensor");
    parameter_boolean_declare_with_default(&publish_imu, &publish_sensor_ns, "imu", false);

    raw_imu_pub.construct<Node &>(node);
    int res = raw_imu_pub->init();
    if (res < 0) {
        chSysHalt("bad res");
    }

    static THD_WORKING_AREA(thd_wa, 256);
    chThdCreateStatic(thd_wa, sizeof(thd_wa),
                      NORMALPRIO,
                      topics_watcher, NULL);
}

void topics_publisher_spin(Node &node)
{
    (void) node;

    if (chBSemWaitTimeout(&imu_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t *topic;
        imu_msg_t imu_data;
        topic = messagebus_find_topic(&bus, "/imu");
        messagebus_topic_read(topic, &imu_data, sizeof(imu_data));

        uavcan::equipment::ahrs::RawIMU msg;
        msg.integration_interval = -1;
        msg.timestamp.usec = imu_data.timestamp;

        msg.rate_gyro_latest[0] = imu_data.gyro.x;
        msg.rate_gyro_latest[1] = imu_data.gyro.y;
        msg.rate_gyro_latest[2] = imu_data.gyro.z;

        msg.accelerometer_latest[0] = imu_data.acc.x;
        msg.accelerometer_latest[1] = imu_data.acc.y;
        msg.accelerometer_latest[2] = imu_data.acc.z;

        raw_imu_pub->broadcast(msg);
    }
}
