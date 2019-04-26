#include "electron_starter.hpp"

#include <cvra/uwb_beacon/DataPacket.hpp>
#include "config.h"

using DataPacket = cvra::uwb_beacon::DataPacket;

static uavcan::LazyConstructor<uavcan::Publisher<DataPacket>> data_pub;

static bool electron_started = false;

int electron_starter_init(uavcan::INode& node)
{
    data_pub.construct<uavcan::INode&>(node);
    static uavcan::Timer periodic_timer(node);

    periodic_timer.setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;

            if (!electron_started) {
                return;
            }

            auto tx_packet = DataPacket();
            tx_packet.dst_addr = config_get_integer("/master/electron_uwb_mac");

            data_pub->broadcast(tx_packet);
        });

    /* Send a message to the electron every second */
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000));

    return 0;
}

void electron_starter_start(void)
{
    electron_started = true;
}
