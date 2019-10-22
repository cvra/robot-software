#include "data_packet_handler.hpp"
#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/DataPacket.hpp>

#define UWB_MAC_START_SIGNAL 42

using DataPacket = cvra::uwb_beacon::DataPacket;

static bool start_received = false;

bool data_packet_start_signal_received(void)
{
    return start_received;
}

static void data_packet_cb(const uavcan::ReceivedDataStructure<DataPacket>& msg)
{
    if (msg.dst_addr == UWB_MAC_START_SIGNAL) {
        start_received = true;
    }
}

int data_packet_handler_init(uavcan::INode& node)
{
    static uavcan::Subscriber<DataPacket> subscriber(node);

    return subscriber.start(data_packet_cb);
}
