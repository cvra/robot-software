#ifndef DATA_PACKET_HANDLER_HPP
#define DATA_PACKET_HANDLER_HPP

#include <uavcan/uavcan.hpp>

int data_packet_handler_init(uavcan::INode& node);

bool data_packet_start_signal_received(void);

#endif
