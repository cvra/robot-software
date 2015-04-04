#include <ch.h>
#include <hal.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/Reboot.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <control.h>
#include <encoder.h>
#include "uavcan_node.h"
#include <can-bootloader/boot_arg.h>

#define CAN_BITRATE 1000000

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<4096> Node;

uavcan::LazyConstructor<Node> node_;

Node& get_node()
{
    if (!node_.isConstructed()) {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void uavcan_failure(const char *reason)
{
    chSysHalt(reason);
}

static THD_WORKING_AREA(uavcan_node_wa, 4000);
static THD_FUNCTION(uavcan_node, arg)
{
    struct uavcan_node_arg *node_arg;
    node_arg = (struct uavcan_node_arg *)arg;

    chRegSetThreadName("uavcan node");

    if (can.init(CAN_BITRATE) != 0) {
        uavcan_failure("CAN driver");
    }

    Node& node = get_node();

    node.setNodeID(node_arg->node_id);
    node.setName(node_arg->node_name);

    if (node.start() != 0) {
        uavcan_failure("UAVCAN node start");
    }

    /* Subscribers */
    uavcan::Subscriber<cvra::Reboot> reboot_sub(node);
    int ret = reboot_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::Reboot>& msg)
        {
            switch (msg.bootmode) {
            case msg.REBOOT:
                reboot(BOOT_ARG_START_APPLICATION);
                break;
            case msg.BOOTLOADER_TIMEOUT:
                reboot(BOOT_ARG_START_BOOTLOADER);
                break;
            case msg.BOOTLOADER_NO_TIMEOUT:
                reboot(BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT);
                break;
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::Reboot subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Velocity> vel_ctrl_sub(node);
    ret = vel_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Velocity>& msg)
        {
            control_update_velocity_setpoint(msg.velocity);
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Velocity subscriber");
    }

    /* Publishers */
    uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> enc_pos_pub(node);
    const int enc_pos_pub_init_res = enc_pos_pub.init();
    if (enc_pos_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorEncoderPosition publisher");
    }


    node.setStatusOk();

    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(100));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        cvra::motor::feedback::MotorEncoderPosition enc_pos;
        enc_pos.raw_encoder_position = encoder_get_secondary();
        enc_pos_pub.broadcast(enc_pos);

    }
    return 0;
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
