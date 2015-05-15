#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <main.h>
#include <uavcan/uavcan.hpp>
#include <cvra/Reboot.hpp>
#include <control.h>
#include <parameter/parameter.h>
#include <encoder.h>
#include "timestamp/timestamp.h"
#include "uavcan_node.h"
#include <can-bootloader/boot_arg.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/TorqueLimit.hpp>
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/StringID.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Trajectory.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

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

static THD_WORKING_AREA(uavcan_node_wa, 8000);
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

    uavcan::Subscriber<cvra::motor::control::Trajectory> traj_ctrl_sub(node);
    ret = traj_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Trajectory>& msg)
        {
            timestamp_t timestamp = timestamp_get();
            control_update_trajectory_setpoint(msg.position,
                                               msg.velocity,
                                               msg.acceleration,
                                               msg.torque,
                                               timestamp);
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Trajectory subscriber");
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

    uavcan::Subscriber<cvra::motor::control::Torque> torque_ctrl_sub(node);
    ret = torque_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Torque>& msg)
        {
            control_update_torque_setpoint(msg.torque);
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Torque subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Voltage> voltage_ctrl_sub(node);
    ret = voltage_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Voltage>& msg)
        {
            (void)msg;
            chSysHalt("voltage control not implemented yet");
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Voltage subscriber");
    }

    /* Publishers */
    uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> enc_pos_pub(node);
    const int enc_pos_pub_init_res = enc_pos_pub.init();
    if (enc_pos_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorEncoderPosition publisher");
    }

    uavcan::Publisher<cvra::motor::config::StringID> string_id_pub(node);
    const int string_id_pub_init_res = string_id_pub.init();
    if (string_id_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::config::StringID publisher");
    }


    /* Servers */
    /** initial config */
    uavcan::ServiceServer<cvra::motor::config::LoadConfiguration> load_config_srv(node);
    const int load_config_srv_res = load_config_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::LoadConfiguration::Request>& req,
            cvra::motor::config::LoadConfiguration::Response& rsp)
        {
            control_stop();

            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/acceleration_limit"), req.acceleration_limit);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/velocity_limit"), req.velocity_limit);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/torque_limit"), req.torque_limit);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/low_batt_th"), req.low_batt_th);

            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/current/kp"), req.current_pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/current/ki"), req.current_pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/current/kd"), req.current_pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/current/i_limit"), req.current_pid.ilimit);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/velocity/kp"), req.velocity_pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/velocity/ki"), req.velocity_pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/velocity/kd"), req.velocity_pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/velocity/i_limit"), req.velocity_pid.ilimit);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/position/kp"), req.position_pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/position/ki"), req.position_pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/position/kd"), req.position_pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "control/position/i_limit"), req.position_pid.ilimit);

            parameter_scalar_set(parameter_find(&parameter_root_ns, "motor/torque_cst"), req.torque_constant);

            parameter_scalar_set(parameter_find(&parameter_root_ns, "thermal/current_gain"), req.thermal_current_gain);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "thermal/max_temp"), req.max_temperature);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "thermal/Rth"), req.thermal_resistance);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "thermal/Cth"), req.thermal_capacity);

            rsp.error_message = "";

            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_OPEN_LOOP) {
                rsp.error_message = "not implemented yet";
            }
            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_INDEX) {
                control_feedback.input_selection = FEEDBACK_RPM;
            }
            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_ENC_PERIODIC) {
                control_feedback.input_selection = FEEDBACK_PRIMARY_ENCODER_PERIODIC;
            }
            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_ENC_BOUNDED) {
                control_feedback.input_selection = FEEDBACK_PRIMARY_ENCODER_BOUNDED;
            }
            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_2_ENC_PERIODIC) {
                control_feedback.input_selection = FEEDBACK_TWO_ENCODERS_PERIODIC;
            }
            if (req.mode == cvra::motor::config::LoadConfiguration::Request::MODE_MOTOR_POT) {
                control_feedback.input_selection = FEEDBACK_POTENTIOMETER;
            }


            control_feedback.primary_encoder.transmission_p = req.transmission_ratio_p;
            control_feedback.primary_encoder.transmission_q = req.transmission_ratio_q;
            control_feedback.primary_encoder.ticks_per_rev = req.motor_encoder_steps_per_revolution;

            control_feedback.secondary_encoder.transmission_p = 1;
            control_feedback.secondary_encoder.transmission_q = 1;
            control_feedback.secondary_encoder.ticks_per_rev = req.second_encoder_steps_per_revolution;

            control_feedback.potentiometer.gain = req.potentiometer_gain;
            control_feedback.potentiometer.zero = 0;

            control_feedback.rpm.phase = 0;

            control_start();

            chprintf(ch_stdout, "LoadConfiguration received\n");

            node.setStatusOk();
        });

    if (load_config_srv_res < 0) {
        uavcan_failure("cvra::motor::config::LoadConfiguration server");
    }

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

    /** Velocity PID config */
    uavcan::ServiceServer<cvra::motor::config::VelocityPID> velocity_pid_srv(node);
    const int velocity_pid_srv_res = velocity_pid_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::VelocityPID::Request>& req,
            cvra::motor::config::VelocityPID::Response& rsp)
        {
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/i_limit"), req.pid.ilimit);
            rsp.dummy = 1;
        });

    if (velocity_pid_srv_res < 0) {
        uavcan_failure("cvra::motor::config::VelocityPID server");
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

    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(100));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        cvra::motor::feedback::MotorEncoderPosition enc_pos;
        enc_pos.raw_encoder_position = encoder_get_secondary();
        enc_pos_pub.broadcast(enc_pos);

        cvra::motor::config::StringID string_id;
        string_id.id = node_arg->node_name;
        string_id_pub.broadcast(string_id);

    }
    return 0;
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
