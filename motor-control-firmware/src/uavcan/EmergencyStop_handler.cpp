#include "EmergencyStop_handler.hpp"
#include <can-bootloader/boot_arg.h>
#include <cvra/motor/EmergencyStop.hpp>

int EmergencyStop_handler_start(Node &node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::EmergencyStop> sub(node);

    ret =  sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop> &msg)
    {
        (void)msg;
        reboot_system(BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT);
    });
    return ret;
}
