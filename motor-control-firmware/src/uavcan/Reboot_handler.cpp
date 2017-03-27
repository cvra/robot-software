#include "Reboot_handler.hpp"
#include <can-bootloader/boot_arg.h>

void Reboot_handler(const uavcan::ReceivedDataStructure<cvra::Reboot> &msg)
{
    switch (msg.bootmode) {
    case msg.REBOOT:
        reboot_system(BOOT_ARG_START_APPLICATION);
        break;
    case msg.BOOTLOADER_TIMEOUT:
        reboot_system(BOOT_ARG_START_BOOTLOADER);
        break;
    case msg.BOOTLOADER_NO_TIMEOUT:
        reboot_system(BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT);
        break;
   }
}
