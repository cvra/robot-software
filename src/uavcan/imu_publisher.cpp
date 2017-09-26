#include <ch.h>
#include <hal.h>
#include "main.h"
#include "uavcan/imu_publisher.hpp"
#include "imu_thread.h"

#include <uavcan/equipment/ahrs/RawIMU.hpp>

uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::ahrs::RawIMU> > raw_imu_pub;

static BSEMAPHORE_DECL(topic_signaled, true);

static parameter_t publish_imu;
static parameter_namespace_t publish_sensor_ns;

static void imu_watcher(void *p)
{
    (void) p;
    messagebus_topic_t *topic;
    imu_msg_t msg;

    chRegSetThreadName(__FUNCTION__);
    topic = messagebus_find_topic_blocking(&bus, "/imu");

    while (1) {
        if (parameter_boolean_get(&publish_imu)) {
            messagebus_topic_wait(topic, &msg, sizeof(msg));
            chBSemSignal(&topic_signaled);
        } else {
            /* Sleep to avoid using CPU. */
            chThdSleepMilliseconds(100);
        }
    }
}

void imu_publisher_start(Node &node)
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
                      imu_watcher, NULL);
}

void imu_publisher_spin(Node &node)
{
    (void) node;

    if (chBSemWaitTimeout(&topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t *topic;
        imu_msg_t imu_data;
        topic = messagebus_find_topic(&bus, "/imu");
        messagebus_topic_read(topic, &imu_data, sizeof(imu_data));

        uavcan::equipment::ahrs::RawIMU msg;
        msg.integration_interval = -1;

        msg.rate_gyro_latest[0] = imu_data.gyro.x;
        msg.rate_gyro_latest[1] = imu_data.gyro.y;
        msg.rate_gyro_latest[2] = imu_data.gyro.z;

        msg.accelerometer_latest[0] = imu_data.acc.x;
        msg.accelerometer_latest[1] = imu_data.acc.y;
        msg.accelerometer_latest[2] = imu_data.acc.z;

        raw_imu_pub->broadcast(msg);
    }
}
