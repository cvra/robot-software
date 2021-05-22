#include "beacon_signal_handler.hpp"
#include <cvra/proximity_beacon/Signal.hpp>
#include <error/error.h>
#include <timestamp/timestamp.h>

#include "main.h"
#include "config.h"
#include "robot_helpers/beacon_helpers.h"
#include "protobuf/beacons.pb.h"

static TOPIC_DECL(proximity_beacon_topic, BeaconSignal);

static void beacon_cb(const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg)
{
    float reflector_radius;
    float angular_offset;

    parameter_t *reflector_radius_p, *angular_offset_p;
    reflector_radius_p = parameter_find(&global_config, "/master/beacon/reflector_radius");
    angular_offset_p = parameter_find(&global_config, "/master/beacon/angular_offset");

    if (reflector_radius_p == NULL || angular_offset_p == NULL) {
        WARNING("Cound not find beacon parameters!");
        return;
    }

    reflector_radius = parameter_scalar_get(reflector_radius_p);
    angular_offset = parameter_scalar_get(angular_offset_p);

    BeaconSignal data;

    // TODO: replace this once the timestamp module is ported
    // TODO: Keep a running buffer of 2 samples in case there are multiple opponents.
    data.timestamp.us = 0;
    data.range.range.distance = reflector_radius / tanf(msg.length / 2.);
    data.range.range.type = Range_RangeType_OTHER;
    data.range.angle = beacon_get_angle(msg.start_angle + angular_offset, msg.length);

    beacon_cartesian_convert(&robot.pos,
                             data.range.range.distance, data.range.angle,
                             &data.x, &data.y);
    messagebus_topic_publish(&proximity_beacon_topic.topic, &data, sizeof(data));

    DEBUG("Opponent detected at: %.3fm, %.3frad \traw signal: %.3f, %.3f",
          data.range.range.distance,
          data.range.angle, msg.start_angle, msg.length);
}

int beacon_signal_handler_init(uavcan::INode& node)
{
    messagebus_advertise_topic(&bus, &proximity_beacon_topic.topic, "/proximity_beacon");

    static uavcan::Subscriber<cvra::proximity_beacon::Signal> prox_beac_sub(node);

    return prox_beac_sub.start(beacon_cb);
}
