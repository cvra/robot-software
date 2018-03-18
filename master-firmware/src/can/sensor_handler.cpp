#include <ch.h>
#include <uavcan/uavcan.hpp>
#include <cvra/sensor/DistanceVL6180X.hpp>

#include <error/error.h>
#include "main.h"

static messagebus_topic_t hand_distance_topic;
static MUTEX_DECL(hand_distance_topic_lock);
static CONDVAR_DECL(hand_distance_topic_condvar);
static float hand_distance_topic_value;

static void sensor_distance_cb(const uavcan::ReceivedDataStructure<cvra::sensor::DistanceVL6180X>& msg)
{
    if (msg.status == cvra::sensor::DistanceVL6180X::STATUS_OK) {
        float dist = (float)msg.distance_mm / 1000.0f;
        messagebus_topic_publish(&hand_distance_topic, &dist, sizeof(dist));
    } else {
        DEBUG("Hand distance measurement error: %d", (int)msg.status);
    }
}

int sensor_handler_init(uavcan::INode &node)
{
    messagebus_topic_init(&hand_distance_topic,
                          &hand_distance_topic_lock,
                          &hand_distance_topic_condvar,
                          &hand_distance_topic_value,
                          sizeof(hand_distance_topic_value));

    messagebus_advertise_topic(&bus, &hand_distance_topic, "/hand_distance");

    static uavcan::Subscriber<cvra::sensor::DistanceVL6180X> distance_sub(node);

    return distance_sub.start(sensor_distance_cb);
}

