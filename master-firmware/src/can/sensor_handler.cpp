#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/sensor/DistanceVL6180X.hpp>

#include <error/error.h>
#include "main.h"
#include "protobuf/sensors.pb.h"
#include <msgbus_protobuf.h>

static TOPIC_DECL(front_left_topic, Range);
static TOPIC_DECL(front_right_topic, Range);
static TOPIC_DECL(back_left_topic, Range);
static TOPIC_DECL(back_right_topic, Range);

static bus_enumerator_t* enumerator;

static void sensor_distance_cb(const uavcan::ReceivedDataStructure<cvra::sensor::DistanceVL6180X>& msg)
{
    int id = msg.getSrcNodeID().get();
    messagebus_topic_t* topic = NULL;

    if (id == bus_enumerator_get_can_id(enumerator, "front-left-sensor")) {
        topic = &front_left_topic.topic;
    } else if (id == bus_enumerator_get_can_id(enumerator, "front-right-sensor")) {
        topic = &front_right_topic.topic;
    } else if (id == bus_enumerator_get_can_id(enumerator, "back-left-sensor")) {
        topic = &back_left_topic.topic;
    } else if (id == bus_enumerator_get_can_id(enumerator, "back-right-sensor")) {
        topic = &back_right_topic.topic;
    }

    if (topic) {
        Range dist;
        dist.distance = msg.distance_mm / 1000.f;
        dist.type = Range_RangeType_LASER;
        messagebus_topic_publish(topic, &dist, sizeof(dist));
    }
}

int sensor_handler_init(uavcan::INode& node, bus_enumerator_t* e)
{
    enumerator = e;

    messagebus_advertise_topic(&bus, &front_left_topic.topic, "/distance/front_left");
    messagebus_advertise_topic(&bus, &front_right_topic.topic, "/distance/front_right");
    messagebus_advertise_topic(&bus, &back_left_topic.topic, "/distance/back_left");
    messagebus_advertise_topic(&bus, &back_right_topic.topic, "/distance/back_right");

    static uavcan::Subscriber<cvra::sensor::DistanceVL6180X> distance_sub(node);

    return distance_sub.start(sensor_distance_cb);
}
