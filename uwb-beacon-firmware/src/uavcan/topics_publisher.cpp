#include <ch.h>
#include <hal.h>
#include "main.h"
#include "topics_publisher.hpp"
#include "imu_thread.h"
#include "ahrs_thread.h"
#include "ranging_thread.h"
#include "state_estimation_thread.h"

#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/ahrs/Solution.hpp>
#include <cvra/uwb_beacon/RadioRange.hpp>
#include <cvra/uwb_beacon/TagPosition.hpp>
#include <cvra/uwb_beacon/DataPacket.hpp>

#define WATCHED_TOPIC_CNT 4

using RadioRange = cvra::uwb_beacon::RadioRange;
using TagPosition = cvra::uwb_beacon::TagPosition;
using DataPacket = cvra::uwb_beacon::DataPacket;

uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::ahrs::RawIMU>> raw_imu_pub;
uavcan::LazyConstructor<uavcan::Publisher<uavcan::equipment::ahrs::Solution>> attitude_pub;
uavcan::LazyConstructor<uavcan::Publisher<RadioRange>> range_pub;
uavcan::LazyConstructor<uavcan::Publisher<TagPosition>> tag_pos_pub;
uavcan::LazyConstructor<uavcan::Publisher<DataPacket>> data_packets_pub;

static BSEMAPHORE_DECL(imu_topic_signaled, true);
static BSEMAPHORE_DECL(attitude_topic_signaled, true);
static BSEMAPHORE_DECL(range_topic_signaled, true);
static BSEMAPHORE_DECL(tag_pos_topic_signaled, true);
static BSEMAPHORE_DECL(data_packet_topic_signaled, true);

static parameter_t publish_imu;
static parameter_t publish_attitude;
static parameter_t publish_range;
static parameter_t publish_tag_pos;
static parameter_t publish_data_packets;
static parameter_namespace_t publish_ns;

static void topics_watcher(void* p)
{
    (void)p;

    chRegSetThreadName(__FUNCTION__);

    static messagebus_watcher_t watchers[WATCHED_TOPIC_CNT];

    static struct {
        mutex_t lock;
        condition_variable_t cv;
        messagebus_watchgroup_t group;
    } watchgroup;

    static messagebus_topic_t *imu_topic, *attitude_topic, *range_topic, *tag_pos_topic, *data_packet_topic;

    /* Create a watchgroup. */
    chMtxObjectInit(&watchgroup.lock);
    chCondObjectInit(&watchgroup.cv);

    messagebus_watchgroup_init(&watchgroup.group, &watchgroup.lock, &watchgroup.cv);

    /* Adds all the topics to the watchers. */
    /* TODO: If a topic is missing, the thread will hang forever.
     * Maybe it would be better to check in the loop if the topic exists. */
    //imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

    tag_pos_topic = messagebus_find_topic_blocking(&bus, "/ekf/state");
    messagebus_watchgroup_watch(&watchers[0],
                                &watchgroup.group,
                                tag_pos_topic);

    data_packet_topic = messagebus_find_topic_blocking(&bus, "/uwb_data");
    messagebus_watchgroup_watch(&watchers[1],
                                &watchgroup.group,
                                data_packet_topic);

    while (1) {
        /* Wait for a topic publication. */
        messagebus_topic_t* topic;
        topic = messagebus_watchgroup_wait(&watchgroup.group);

        if (topic == imu_topic && parameter_boolean_get(&publish_imu)) {
            chBSemSignal(&imu_topic_signaled);
        } else if (topic == attitude_topic && parameter_boolean_get(&publish_attitude)) {
            chBSemSignal(&attitude_topic_signaled);
        } else if (topic == range_topic && parameter_boolean_get(&publish_range)) {
            chBSemSignal(&range_topic_signaled);
        } else if (topic == tag_pos_topic && parameter_boolean_get(&publish_tag_pos)) {
            chBSemSignal(&tag_pos_topic_signaled);
        } else if (topic == data_packet_topic && parameter_boolean_get(&publish_data_packets)) {
            chBSemSignal(&data_packet_topic_signaled);
        }
    }
}

void topics_publisher_start(Node& node)
{
    parameter_namespace_declare(&publish_ns, &parameter_root, "publish");
    parameter_boolean_declare_with_default(&publish_imu, &publish_ns, "imu", false);
    parameter_boolean_declare_with_default(&publish_attitude, &publish_ns, "attitude", false);
    parameter_boolean_declare_with_default(&publish_range, &publish_ns, "range", false);
    parameter_boolean_declare_with_default(&publish_tag_pos, &publish_ns, "tag_positions", false);
    parameter_boolean_declare_with_default(&publish_data_packets, &publish_ns, "data_packets", true);

    raw_imu_pub.construct<Node&>(node);
    int res = raw_imu_pub->init();
    chDbgAssert(res == 0, "imu publisher failed");

    attitude_pub.construct<Node&>(node);
    res = attitude_pub->init();
    chDbgAssert(res == 0, "attitude publisher failed");

    range_pub.construct<Node&>(node);
    res = range_pub->init();
    chDbgAssert(res == 0, "range publisher failed");

    tag_pos_pub.construct<Node&>(node);
    res = tag_pos_pub->init();
    chDbgAssert(res == 0, "pos publisher failed");

    data_packets_pub.construct<Node&>(node);
    res = data_packets_pub->init();
    chDbgAssert(res == 0, "data publisher failed");

    static THD_WORKING_AREA(thd_wa, 256);
    chThdCreateStatic(thd_wa, sizeof(thd_wa),
                      NORMALPRIO,
                      topics_watcher, NULL);
}

void topics_publisher_spin(Node& node)
{
    (void)node;

    if (chBSemWaitTimeout(&imu_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t* topic;
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

    if (chBSemWaitTimeout(&attitude_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t* topic;
        attitude_msg_t attitude;

        topic = messagebus_find_topic(&bus, "/attitude");
        messagebus_topic_read(topic, &attitude, sizeof(attitude));

        uavcan::equipment::ahrs::Solution msg;
        msg.timestamp.usec = attitude.timestamp;
        msg.orientation_xyzw[0] = attitude.q.x;
        msg.orientation_xyzw[1] = attitude.q.y;
        msg.orientation_xyzw[2] = attitude.q.z;
        msg.orientation_xyzw[3] = attitude.q.w;
        attitude_pub->broadcast(msg);
    }

    if (chBSemWaitTimeout(&range_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t* topic;
        range_msg_t range_msg;

        topic = messagebus_find_topic(&bus, "/range");
        messagebus_topic_read(topic, &range_msg, sizeof(range_msg));

        RadioRange msg;
        msg.range = range_msg.range;
        msg.anchor_addr = range_msg.anchor_addr;
        msg.timestamp.usec = range_msg.timestamp;
        range_pub->broadcast(msg);
    }

    if (chBSemWaitTimeout(&tag_pos_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t* topic;
        position_estimation_msg_t tag_pos_msg;

        topic = messagebus_find_topic(&bus, "/ekf/state");
        messagebus_topic_read(topic, &tag_pos_msg, sizeof(tag_pos_msg));

        TagPosition msg;
        msg.timestamp.usec = tag_pos_msg.timestamp;
        msg.x = tag_pos_msg.x;
        msg.y = tag_pos_msg.y;
        tag_pos_pub->broadcast(msg);
    }

    if (chBSemWaitTimeout(&data_packet_topic_signaled, TIME_IMMEDIATE) == MSG_OK) {
        messagebus_topic_t* topic;
        static data_packet_msg_t data_msg;

        topic = messagebus_find_topic(&bus, "/uwb_data");
        messagebus_topic_read(topic, &data_msg, sizeof(data_msg));

        DataPacket msg;
        msg.src_addr = data_msg.src_mac;
        msg.dst_addr = data_msg.dst_mac;

        for (auto i = 0u; i < data_msg.data_size; i++) {
            msg.data.push_back(data_msg.data[i]);
        }
        data_packets_pub->broadcast(msg);
    }
}
