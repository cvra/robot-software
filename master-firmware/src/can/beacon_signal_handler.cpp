#include "beacon_signal_handler.hpp"
#include <cvra/proximity_beacon/Signal.hpp>
#include <error/error.h>
#include <timestamp/timestamp.h>

#include "main.h"
#include "config.h"
#include "robot_helpers/beacon_helpers.h"

static messagebus_topic_t proximity_beacon_topic;
static MUTEX_DECL(proximity_beacon_topic_lock);
static CONDVAR_DECL(proximity_beacon_topic_condvar);
static beacon_signal_t proximity_beacon_topic_value;

int beacon_signal_handler_init(uavcan::INode &node)
{
    messagebus_topic_init(&proximity_beacon_topic,
                          &proximity_beacon_topic_lock,
                          &proximity_beacon_topic_condvar,
                          &proximity_beacon_topic_value,
                          sizeof(proximity_beacon_topic_value));

    messagebus_advertise_topic(&bus, &proximity_beacon_topic, "/proximity_beacon");

    static uavcan::Subscriber<cvra::proximity_beacon::Signal> prox_beac_sub(node);
    int res = prox_beac_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg)
    {
        float reflector_radius;
        float angular_offset;

        parameter_t *reflector_radius_p, *angular_offset_p;
        reflector_radius_p = parameter_find(&global_config, "/master/beacon/reflector_radius");
        angular_offset_p = parameter_find(&global_config, "/master/beacon/angular_offset");

        if (reflector_radius_p == NULL || angular_offset_p == NULL) {
            ERROR("Cound not find beacon parameters!");
            return;
        }

        reflector_radius = parameter_scalar_get(reflector_radius_p);
        angular_offset = parameter_scalar_get(angular_offset_p);

        beacon_signal_t data;
        data.timestamp = timestamp_get();
        data.distance = reflector_radius + reflector_radius / tanf(msg.length / 2.);
        data.heading = beacon_get_angle(msg.start_angle + angular_offset, msg.length);
        messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

        DEBUG("Opponent detected at: %.3fm, %.3frad \traw signal: %.3f, %.3f", data.distance,
              data.heading, msg.start_angle, msg.length);
    });

    return res;
}
