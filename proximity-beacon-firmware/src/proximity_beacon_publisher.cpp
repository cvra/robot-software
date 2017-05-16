#include <cvra/proximity_beacon/Signal.hpp>
#include <cvra/proximity_beacon/Settings.hpp>
#include "proximity_beacon.h"

static uavcan::LazyConstructor<uavcan::Publisher<cvra::proximity_beacon::Signal> > signal_pub;

static void timer_cb(const uavcan::TimerEvent &event)
{
    struct proximity_beacon_signal *pbs;
    while ((pbs = proximity_beacon_signal_get()) != NULL) {
        cvra::proximity_beacon::Signal sig;
        sig.start_angle = pbs->start_angle;
        sig.length = pbs->length;
        prox_beac_pub.broadcast(sig);
        proximity_beacon_signal_delete(pbs);
    }

    /* periodic setpoint update to keep the motor running */
    control_update_voltage_setpoint(-4.0);
}

int proximity_beacon_start(Node &node)
{
    proximity_beacon_init();

    control_stop();
    /* TODO: set needed parameters */
    control_start();

    /* Mark the node as correctly initialized */
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    static uavcan::Timer periodic_timer(node);
    periodic_timer.setCallback(timer_cb);
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));

    signal_pub.construct<uavcan::INode &>(node);
    int res = signal_pub->init();
    if (res < 0) {
        return res;
    }
}

