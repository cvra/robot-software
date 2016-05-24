#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <main.h>
#include <uavcan/uavcan.hpp>
#include <cvra/Reboot.hpp>
#include <control.h>
#include <index.h>
#include <parameter/parameter.h>
#include <encoder.h>
#include "timestamp/timestamp.h"
#include "uavcan_node.h"
#include <can-bootloader/boot_arg.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include "stream.h"
#include <cvra/motor/config/EnableMotor.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/TorqueLimit.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>
#include <cvra/StringID.hpp>
#include <cvra/motor/EmergencyStop.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Trajectory.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

#define CAN_BITRATE             1000000
#define UAVCAN_SPIN_FREQUENCY   100

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<4096> Node;

uavcan::LazyConstructor<Node> node_;


stream_config_t string_id_stream_config     = {false, 0, 0};
stream_config_t current_pid_stream_config   = {false, 0, 0};
stream_config_t velocity_pid_stream_config  = {false, 0, 0};
stream_config_t position_pid_stream_config  = {false, 0, 0};
stream_config_t index_stream_config         = {false, 0, 0};
stream_config_t motor_enc_stream_config     = {false, 0, 0};
stream_config_t motor_pos_stream_config     = {false, 0, 0};
stream_config_t motor_torque_stream_config  = {false, 0, 0};


static void stream_init_from_callback(stream_config_t *stream_config,
                                      const uavcan::ReceivedDataStructure<cvra::motor::config::FeedbackStream::Request>& msg)
{
    if (msg.enabled != 0) {
        stream_set_prescaler(stream_config, msg.frequency, UAVCAN_SPIN_FREQUENCY);
        stream_enable(stream_config, true);
    } else {
        stream_enable(stream_config, false);
    }
}


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

    if (can.init((uavcan::uint32_t)CAN_BITRATE) != 0) {
        uavcan_failure("CAN driver");
    }

    Node& node = get_node();

    node.setNodeID(node_arg->node_id);
    node.setName(node_arg->node_name);

    if (node.start() != 0) {
        uavcan_failure("UAVCAN node start");
    }

    stream_set_prescaler(&string_id_stream_config, 0.5, UAVCAN_SPIN_FREQUENCY);
    stream_enable(&string_id_stream_config, true);

    /* Subscribers */
    uavcan::Subscriber<cvra::Reboot> reboot_sub(node);
    int ret = reboot_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::Reboot>& msg)
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
    );
    if (ret != 0) {
        uavcan_failure("cvra::Reboot subscriber");
    }

    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    ret = emergency_stop_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
        {
            (void)msg;
            reboot_system(BOOT_ARG_START_BOOTLOADER_NO_TIMEOUT);
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::EmergencyStop subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Trajectory> traj_ctrl_sub(node);
    ret = traj_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Trajectory>& msg)
        {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                timestamp_t timestamp = timestamp_get();
                control_update_trajectory_setpoint(msg.position,
                                                   msg.velocity,
                                                   msg.acceleration,
                                                   msg.torque,
                                               timestamp);
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Trajectory subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Velocity> vel_ctrl_sub(node);
    ret = vel_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Velocity>& msg)
        {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_velocity_setpoint(msg.velocity);
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Velocity subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Position> pos_ctrl_sub(node);
    ret = pos_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Position>& msg)
        {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_position_setpoint(msg.position);
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Position subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Torque> torque_ctrl_sub(node);
    ret = torque_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Torque>& msg)
        {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_torque_setpoint(msg.torque);
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Torque subscriber");
    }

    uavcan::Subscriber<cvra::motor::control::Voltage> voltage_ctrl_sub(node);
    ret = voltage_ctrl_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Voltage>& msg)
        {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_voltage_setpoint(msg.voltage);
            }
        }
    );
    if (ret != 0) {
        uavcan_failure("cvra::motor::control::Voltage subscriber");
    }

    /* Publishers */
    uavcan::Publisher<cvra::StringID> string_id_pub(node);
    const int string_id_pub_init_res = string_id_pub.init();
    if (string_id_pub_init_res < 0)
    {
        uavcan_failure("cvra::StringID publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::CurrentPID> current_pid_pub(node);
    const int current_pid_pub_init_res = current_pid_pub.init();
    if (current_pid_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::CurrentPID publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::VelocityPID> velocity_pid_pub(node);
    const int velocity_pid_pub_init_res = velocity_pid_pub.init();
    if (velocity_pid_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::VelocityPID publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::PositionPID> position_pid_pub(node);
    const int position_pid_pub_init_res = position_pid_pub.init();
    if (position_pid_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::PositionPID publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::Index> index_pub(node);
    const int index_pub_init_res = index_pub.init();
    if (index_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::Index publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> enc_pos_pub(node);
    const int enc_pos_pub_init_res = enc_pos_pub.init();
    if (enc_pos_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorEncoderPosition publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::MotorPosition> motor_pos_pub(node);
    const int motor_pos_pub_init_res = motor_pos_pub.init();
    if (motor_pos_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorPosition publisher");
    }

    uavcan::Publisher<cvra::motor::feedback::MotorTorque> motor_torque_pub(node);
    const int motor_torque_pub_init_res = motor_torque_pub.init();
    if (motor_torque_pub_init_res < 0)
    {
        uavcan_failure("cvra::motor::feedback::MotorTorque publisher");
    }



    /* Servers */
    /** initial config */
    uavcan::ServiceServer<cvra::motor::config::LoadConfiguration> load_config_srv(node);
    const int load_config_srv_res = load_config_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::LoadConfiguration::Request>& req,
                                                cvra::motor::config::LoadConfiguration::Response& rsp)
        {
            (void) rsp;

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

            // Mark the node as correctly initialized
            node.getNodeStatusProvider().setModeOperational();
            node.getNodeStatusProvider().setHealthOk();
        });

    if (load_config_srv_res < 0) {
        uavcan_failure("cvra::motor::config::LoadConfiguration server");
    }

    /** Feedback Stream config*/
    uavcan::ServiceServer<cvra::motor::config::FeedbackStream> feedback_stream_sub(node);
    const int feedback_stream_srv_res = feedback_stream_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::FeedbackStream::Request>& req,
                                                cvra::motor::config::FeedbackStream::Response& rsp)
        {

            (void) rsp;

            switch (req.stream) {
                case cvra::motor::config::FeedbackStream::Request::STREAM_CURRENT_PID : {
                    stream_init_from_callback(&current_pid_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_VELOCITY_PID : {
                    stream_init_from_callback(&velocity_pid_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_POSITION_PID : {
                    stream_init_from_callback(&position_pid_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_INDEX : {
                    stream_init_from_callback(&index_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_ENCODER : {
                    stream_init_from_callback(&motor_enc_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_POSITION : {
                    stream_init_from_callback(&motor_pos_stream_config, req);
                    break;
                }
                case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_TORQUE : {
                    stream_init_from_callback(&motor_torque_stream_config, req);
                    break;
                }
            }
        }
    );
    if (feedback_stream_srv_res < 0) {
        uavcan_failure("cvra::motor::config::FeedbackStream server");
    }

    /** Current PID config */
    uavcan::ServiceServer<cvra::motor::config::CurrentPID> current_pid_srv(node);
    const int current_pid_srv_res = current_pid_srv.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::CurrentPID::Request>& req,
                                                cvra::motor::config::CurrentPID::Response& rsp)
        {
            (void) rsp; /* empty response */
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/current/i_limit"), req.pid.ilimit);
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
            (void) rsp;

            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/velocity/i_limit"), req.pid.ilimit);
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
            (void) rsp;

            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/kp"), req.pid.kp);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/ki"), req.pid.ki);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/kd"), req.pid.kd);
            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/position/i_limit"), req.pid.ilimit);
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
            (void) rsp;

            parameter_scalar_set(parameter_find(&parameter_root_ns, "/control/torque_limit"), req.torque_limit);
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
            (void) rsp;     /* empty response */
            control_enable(req.enable);
        });

    if (enable_motor_srv_res < 0) {
        uavcan_failure("cvra::motor::config::EnableMotor server");
    }

    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQUENCY));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        /* Streams */
        if (stream_update(&current_pid_stream_config)) {
            cvra::motor::feedback::CurrentPID current_pid;
            current_pid.current = control_get_current();
            current_pid.current_setpoint = control_get_current_setpoint();
            current_pid.motor_voltage = control_get_motor_voltage();
            current_pid_pub.broadcast(current_pid);
        }

        if (stream_update(&velocity_pid_stream_config)) {
            cvra::motor::feedback::VelocityPID velocity_pid;
            velocity_pid.velocity = control_get_velocity();
            velocity_pid.velocity_setpoint = control_get_velocity_setpoint();
            velocity_pid_pub.broadcast(velocity_pid);
        }

        if (stream_update(&position_pid_stream_config)) {
            cvra::motor::feedback::PositionPID position_pid;
            position_pid.position = control_get_position();
            position_pid.position_setpoint = control_get_position_setpoint();
            position_pid_pub.broadcast(position_pid);
        }

        if (stream_update(&motor_enc_stream_config)) {
            cvra::motor::feedback::MotorEncoderPosition enc_pos;
            enc_pos.raw_encoder_position = encoder_get_secondary();
            enc_pos_pub.broadcast(enc_pos);
        }

        if (stream_update(&index_stream_config)) {
            cvra::motor::feedback::Index index;
            float index_pos;
            uint32_t update_count;
            index_get_position(&index_pos, &update_count);
            index.position = index_pos;
            index.update_count = update_count;
            index_pub.broadcast(index);
        }

        if (stream_update(&motor_pos_stream_config)) {
            cvra::motor::feedback::MotorPosition motor_pos;
            motor_pos.position = control_get_position();
            motor_pos.velocity = control_get_velocity();
            motor_pos_pub.broadcast(motor_pos);
        }

        if (stream_update(&motor_torque_stream_config)) {
            cvra::motor::feedback::MotorTorque motor_torque;
            motor_torque.torque = control_get_torque();
            motor_torque.position = control_get_position();
            motor_torque_pub.broadcast(motor_torque);
        }

        if (stream_update(&string_id_stream_config)) {
            cvra::StringID string_id;
            string_id.id = node_arg->node_name;
            string_id_pub.broadcast(string_id);
        }


    }
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
