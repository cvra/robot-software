#include "main.h"
#include "state_estimation_thread.h"
#include "position_handler.hpp"
#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/TagPosition.hpp>

template <typename T>
using Subscriber = uavcan::Subscriber<T>;

template <typename T>
using ReceivedDataStructure = uavcan::ReceivedDataStructure<T>;
using TagPosition = cvra::uwb_beacon::TagPosition;


static void tag_pos_cb(const ReceivedDataStructure<TagPosition>& msg)
{
    auto topic = messagebus_find_topic(&bus, "/ekf/state");

    if (topic == nullptr) {
        return;
    }

    position_estimation_msg_t state_msg;
    state_msg.x = msg.x;
    state_msg.y = msg.y;
    state_msg.variance_x = 0.01;
    state_msg.variance_y = 0.01;

    messagebus_topic_publish(topic, &state_msg, sizeof(state_msg));
}

int position_handler_init(Node &node)
{
    static Subscriber<TagPosition> subscriber(node);

    return subscriber.start(tag_pos_cb);
}
