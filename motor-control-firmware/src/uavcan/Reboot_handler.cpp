#include "Reboot_handler.hpp"
#include <can-bootloader/boot_arg.h>
#include <cvra/Reboot.hpp>

int Reboot_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::Reboot> reboot_sub(node);

    ret = reboot_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::Reboot>& msg) {
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
        });
    return ret;
}
