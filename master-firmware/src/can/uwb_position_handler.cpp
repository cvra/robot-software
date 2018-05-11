#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/TagPosition.hpp>
#include <aversive/math/geometry/vect_base.h>
#include <error/error.h>
#include "main.h"
#include "uwb_position_handler.h"
#include "base/base_controller.h"
#include "uwb_position.h"
#include "control_panel.h"

using TagPosition = cvra::uwb_beacon::TagPosition;

static messagebus_topic_t allied_position_topic;
static MUTEX_DECL(allied_position_topic_lock);
static CONDVAR_DECL(allied_position_topic_condvar);
static allied_position_t allied_position;
static uavcan::LazyConstructor<uavcan::Publisher<TagPosition> > publisher;

static messagebus_topic_t last_panel_contact_topic;
static MUTEX_DECL(last_panel_contact_topic_lock);
static CONDVAR_DECL(last_panel_contact_topic_condvar);
static uint32_t last_panel_contact;

static void position_cb(const uavcan::ReceivedDataStructure<TagPosition> &msg)
{
    allied_position_t pos;
    pos.point.x = msg.x;
    pos.point.y = msg.y;
    pos.timestamp = timestamp_get();

    // This is the panel
    if (msg.x < -1000) {
        uint32_t msg = timestamp_get();
        messagebus_topic_publish(&last_panel_contact_topic, &msg, sizeof(msg));
    } else {
        messagebus_topic_publish(&allied_position_topic, &pos, sizeof(pos));
    }
}

int uwb_position_handler_init(uavcan::INode &node)
{
    messagebus_topic_init(&allied_position_topic,
                          &allied_position_topic_lock,
                          &allied_position_topic_condvar,
                          &allied_position,
                          sizeof(allied_position));

    messagebus_advertise_topic(&bus, &allied_position_topic, "/allied_position");

    messagebus_topic_init(&last_panel_contact_topic,
                          &last_panel_contact_topic_lock,
                          &last_panel_contact_topic_condvar,
                          &last_panel_contact,
                          sizeof(last_panel_contact));

    messagebus_advertise_topic(&bus, &last_panel_contact_topic, "/panel_contact_us");

    static uavcan::Subscriber<TagPosition> position_sub(node);
    auto res = position_sub.start(position_cb);

    if (res != 0) {
        return res;
    }

    publisher.construct<uavcan::INode &>(node);

    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback([](const uavcan::TimerEvent &event)
    {
        (void) event;
        TagPosition msg;
        msg.x = position_get_x_double(&robot.pos);
        msg.y = position_get_y_double(&robot.pos);
        publisher->broadcast(msg);
    });

    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(300));

    return 0;
}
