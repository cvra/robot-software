#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/TagPosition.hpp>
#include <aversive/math/geometry/vect_base.h>
#include <error/error.h>
#include "main.h"
#include "uwb_position_handler.h"
#include "base/base_controller.h"

using TagPosition = cvra::uwb_beacon::TagPosition;

static messagebus_topic_t allied_position_topic;
static MUTEX_DECL(allied_position_topic_lock);
static CONDVAR_DECL(allied_position_topic_condvar);
static point_t allied_position;
static uavcan::LazyConstructor<uavcan::Publisher<TagPosition> > publisher;

static void position_cb(const uavcan::ReceivedDataStructure<TagPosition> &msg)
{
    point_t pos;
    pos.x = msg.x;
    pos.y = msg.y;
    messagebus_topic_publish(&allied_position_topic, &pos, sizeof(pos));
    DEBUG("Received position from allied robot: %.2f %.2f", pos.x, pos.y);
}

int uwb_position_handler_init(uavcan::INode &node)
{
    messagebus_topic_init(&allied_position_topic,
                          &allied_position_topic_lock,
                          &allied_position_topic_condvar,
                          &allied_position,
                          sizeof(allied_position));

    messagebus_advertise_topic(&bus, &allied_position_topic, "/allied_position");

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
