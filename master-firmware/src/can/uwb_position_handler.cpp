#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/TagPosition.hpp>
#include <aversive/math/geometry/vect_base.h>
#include <error/error.h>
#include "main.h"
#include "uwb_position_handler.h"
#include "base/base_controller.h"
#include "control_panel.h"
#include "protobuf/beacons.pb.h"
#include <timestamp/timestamp.h>

using TagPosition = cvra::uwb_beacon::TagPosition;

TOPIC_DECL(allied_position_topic, AlliedPosition);
TOPIC_DECL(last_panel_contact_topic, Timestamp);

static uavcan::LazyConstructor<uavcan::Publisher<TagPosition>> publisher;

static void position_cb(const uavcan::ReceivedDataStructure<TagPosition>& msg)
{
    AlliedPosition pos;
    pos.x = msg.x;
    pos.y = msg.y;
    pos.timestamp.us = timestamp_get();

    // This is the panel
    if (msg.x < -1000) {
        Timestamp msg;
        msg.us = timestamp_get();
        messagebus_topic_publish(&last_panel_contact_topic.topic, &msg, sizeof(msg));
    } else {
        messagebus_topic_publish(&allied_position_topic.topic, &pos, sizeof(pos));
    }
}

int uwb_position_handler_init(uavcan::INode& node)
{
    messagebus_advertise_topic(&bus, &allied_position_topic.topic, "/allied_position");
    messagebus_advertise_topic(&bus, &last_panel_contact_topic.topic, "/panel_contact_us");

    static uavcan::Subscriber<TagPosition> position_sub(node);
    auto res = position_sub.start(position_cb);

    if (res != 0) {
        return res;
    }

    publisher.construct<uavcan::INode&>(node);

    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback([](const uavcan::TimerEvent& event) {
        (void)event;
        TagPosition msg;
        msg.x = position_get_x_double(&robot.pos);
        msg.y = position_get_y_double(&robot.pos);
        publisher->broadcast(msg);
    });

    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(300));

    return 0;
}
