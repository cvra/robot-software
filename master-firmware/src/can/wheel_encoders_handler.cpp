#include <memory>
#include <uavcan/uavcan.hpp>
#include <cvra/odometry/WheelEncoder.hpp>
#include "protobuf/encoders.pb.h"
#include <error/error.h>
#include "main.h"
#include <msgbus/messagebus.h>
#include <msgbus/posix/port.h>

static messagebus_topic_t encoders_topic;
static condvar_wrapper_t wrapper = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};

static WheelEncodersPulse msg_content;

using Subscriber = uavcan::Subscriber<cvra::odometry::WheelEncoder>;

static void WheelEncoder_handler(
    const uavcan::ReceivedDataStructure<cvra::odometry::WheelEncoder>& msg)
{
    DEBUG_EVERY_N(100, "received wheel encoder (%d;%d)", msg.left_encoder_raw, msg.right_encoder_raw);
    WheelEncodersPulse bus_msg;
    bus_msg.left = msg.left_encoder_raw;
    bus_msg.right = msg.right_encoder_raw;
    messagebus_topic_publish(&encoders_topic, &bus_msg, sizeof(bus_msg));
}

int wheel_encoder_handler_init(uavcan::INode& node)
{
    messagebus_topic_init(&encoders_topic, &wrapper, &wrapper, &msg_content, sizeof(msg_content));
    messagebus_advertise_topic(&bus, &encoders_topic, "/encoders");

    static Subscriber sub(node);
    return sub.start(WheelEncoder_handler);
}
