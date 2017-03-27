#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <main.h>
#include <uavcan/uavcan.hpp>
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
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

#include "Reboot_handler.hpp"
#include "EmergencyStop_handler.hpp"
#include "Trajectory_handler.hpp"
#include "Velocity_handler.hpp"
#include "Position_handler.hpp"
#include "Torque_handler.hpp"
#include "Voltage_handler.hpp"
#include "parameter_server.hpp"
#include "LoadConfiguration_server.hpp"
#include "EnableMotor_server.hpp"
#include "CurrentPID_server.hpp"
#include "VelocityPID_server.hpp"
#include "PositionPID_server.hpp"
#include "TorqueLimit_server.hpp"

#define CAN_BITRATE             1000000
#define UAVCAN_SPIN_FREQUENCY   100

uavcan_stm32::CanInitHelper<128> can;

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

    chRegSetThreadName(__FUNCTION__);

    if (can.init((uavcan::uint32_t)CAN_BITRATE) != 0) {
        uavcan_failure("CAN driver");
    }

    Node& node = get_node();

    node.setNodeID(node_arg->node_id);
    node.setName(node_arg->node_name);

    parameter_server_start();

    if (node.start() != 0) {
        uavcan_failure("UAVCAN node start");
    }

    struct {
        int (*start)(Node &);
        const char *name;
    } services[] = {
        {Reboot_handler_start, "Reboot subscriber"},
        {EmergencyStop_handler_start, "Emergency stop subscriber"},
        {Trajectory_handler_start, "cvra::motor::control::Trajectory subscriber"},
        {Velocity_handler_start, "cvra::motor::control::Velocity subscriber"},
        {Position_handler_start, "cvra::motor::control::Position subscriber"},
        {Torque_handler_start, "cvra::motor::control::Torque subscriber"},
        {Voltage_handler_start, "cvra::motor::control::Voltage subscriber"},
        {LoadConfiguration_server_start, "cvra::motor::config::LoadConfiguration server"},
        {CurrentPID_server_start, "cvra::motor::config::CurrentPID server"},
        {VelocityPID_server_start, "cvra::motor::config::VelocityPID server"},
        {PositionPID_server_start, "cvra::motor::config::PositionPID server"},
        {TorqueLimit_server_start, "cvra::motor::config::TorqueLimit server"},
        {EnableMotor_server_start, "cvra::motor::config::EnableMotor server"},
        {NULL, NULL} /* Must be last */
    };

    /* Start all services. */
    for (int i = 0; services[i].start; i++) {
        if (services[i].start(node) < 0) {
            uavcan_failure(services[i].name);
        }
    }

    stream_set_prescaler(&string_id_stream_config, 0.5, UAVCAN_SPIN_FREQUENCY);
    stream_enable(&string_id_stream_config, true);

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

    if (feedback_stream_srv_res != 0) {
        uavcan_failure("cvra::motor::config::FeedbackStream server");
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
