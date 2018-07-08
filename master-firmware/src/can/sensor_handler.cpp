#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/sensor/DistanceVL6180X.hpp>

#include <error/error.h>
#include "main.h"
#include "protobuf/sensors.pb.h"
#include "msgbus_protobuf.h"

static TOPIC_DECL(hand_distance_topic, Range);

static void sensor_distance_cb(const uavcan::ReceivedDataStructure<cvra::sensor::DistanceVL6180X>& msg)
{
    if (msg.status == cvra::sensor::DistanceVL6180X::STATUS_OK) {
        Range dist;
        dist.distance =  msg.distance_mm / 1000.f;
        dist.type = Range_RangeType_LASER;
        messagebus_topic_publish(&hand_distance_topic.topic, &dist, sizeof(dist));
    } else {
        DEBUG("Hand distance measurement error: %d", (int)msg.status);
    }
}

int sensor_handler_init(uavcan::INode &node)
{
    messagebus_advertise_topic(&bus, &hand_distance_topic.topic, "/hand_distance");

    static uavcan::Subscriber<cvra::sensor::DistanceVL6180X> distance_sub(node);

    return distance_sub.start(sensor_distance_cb);
}

