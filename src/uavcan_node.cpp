#include <ch.h>
#include <hal.h>
#include <main.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/Reboot.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <control.h>
#include <parameter/parameter.h>
#include <encoder.h>
#include "uavcan_node.h"
#include <can-bootloader/boot_arg.h>

#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/SpeedPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/TorqueLimit.hpp>
#include <cvra/motor/config/EnableMotor.hpp>

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

    uavcan::Subscriber<cvra::motor::control::Position> pos_ctrl_sub(node);
    ret = pos_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Position>& msg)
        {
            control_update_position_setpoint(msg.position);
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Position subscriber");
    }

    /* Publishers */
    uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> enc_pos_pub(node);
    const int enc_pos_pub_init_res = enc_pos_pub.init();
    if (enc_pos_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorEncoderPosition publisher");
    }


    /* Servers */
    /** Current PID config */
    uavcan::ServiceServer<cvra::motor::config::CurrentPID> current_pid_srv(node);
    const int current_pid_srv_res = current_pid_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::CurrentPID::Request>& req,
            cvra::motor::config::CurrentPID::Response& rsp)
        {
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/i_limit"), req.pid.ilimit);
            rsp.dummy = 1;
        });

    if (current_pid_srv_res < 0) {
        uavcan_failure("cvra::motor::config::CurrentPID server");
    }

    /** Speed PID config */
    uavcan::ServiceServer<cvra::motor::config::SpeedPID> velocity_pid_srv(node);
    const int velocity_pid_srv_res = velocity_pid_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::SpeedPID::Request>& req,
            cvra::motor::config::SpeedPID::Response& rsp)
        {
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/i_limit"), req.pid.ilimit);
            rsp.dummy = 1;
        });

    if (velocity_pid_srv_res < 0) {
        uavcan_failure("cvra::motor::config::SpeedPID server");
    }

    /** Position PID config */
    uavcan::ServiceServer<cvra::motor::config::PositionPID> position_pid_srv(node);
    const int position_pid_srv_res = position_pid_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::PositionPID::Request>& req,
            cvra::motor::config::PositionPID::Response& rsp)
        {
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/i_limit"), req.pid.ilimit);
            rsp.dummy = 1;
        });

    if (position_pid_srv_res < 0) {
        uavcan_failure("cvra::motor::config::PositionPID server");
    }

    /** Torque Limit config */
    uavcan::ServiceServer<cvra::motor::config::TorqueLimit> torque_limit_srv(node);
    const int torque_limit_srv_res = torque_limit_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::TorqueLimit::Request>& req,
            cvra::motor::config::TorqueLimit::Response& rsp)
        {
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/torque_limit"), req.torque_limit);
            rsp.dummy = 1;
        });

    if (torque_limit_srv_res < 0) {
        uavcan_failure("cvra::motor::config::TorqueLimit server");
    }

    /** Enable Motor config */
    uavcan::ServiceServer<cvra::motor::config::EnableMotor> enable_motor_srv(node);
    const int enable_motor_srv_res = enable_motor_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::EnableMotor::Request>& req,
            cvra::motor::config::EnableMotor::Response& rsp)
        {
            control_enable(req.enable);
            rsp.dummy = 1;
        });

    if (enable_motor_srv_res < 0) {
        uavcan_failure("cvra::motor::config::EnableMotor server");
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
