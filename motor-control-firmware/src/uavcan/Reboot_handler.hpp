#ifndef REBOOT_HANDLER_HPP
#define REBOOT_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/Reboot.hpp>

void Reboot_handler(const uavcan::ReceivedDataStructure<cvra::Reboot> &msg);

#endif
