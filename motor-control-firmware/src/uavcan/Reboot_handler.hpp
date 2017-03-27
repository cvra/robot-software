#ifndef REBOOT_HANDLER_HPP
#define REBOOT_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/Reboot.hpp>
#include <can-bootloader/boot_arg.h>

void Reboot_handler(const uavcan::ReceivedDataStructure<cvra::Reboot> &msg);

#endif
