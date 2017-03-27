#include "EmergencyStop_handler.hpp"
#include <can-bootloader/boot_arg.h>

void EmergencyStop_handler(const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop> &msg)
{
    (void)msg;
    reboot_system(BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT);
}
