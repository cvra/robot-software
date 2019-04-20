#include "data_packet_handler.hpp"
#include <uavcan/uavcan.hpp>
#include <cvra/uwb_beacon/DataPacket.hpp>

using DataPacket = cvra::uwb_beacon::DataPacket;

static void data_packet_cb(const uavcan::ReceivedDataStructure<DataPacket>& msg)
{
    // msg.data
    // msg.data.size
    // msg.src_addr
    // msg.dst_addr
    (void) msg;
}

int data_packet_handler_init(uavcan::INode& node)
{
    static uavcan::Subscriber<DataPacket> subscriber(node);

    return subscriber.start(data_packet_cb);
}

