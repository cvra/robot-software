#include "uavcan_streams.hpp"
#include "stream.h"

#include "control.h"
#include "index.h"
#include "encoder.h"

#include <cvra/motor/config/FeedbackStream.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>

stream_config_t current_pid_stream_config   = {false, 0, 0};
stream_config_t velocity_pid_stream_config  = {false, 0, 0};
stream_config_t position_pid_stream_config  = {false, 0, 0};
stream_config_t index_stream_config         = {false, 0, 0};
stream_config_t motor_enc_stream_config     = {false, 0, 0};
stream_config_t motor_pos_stream_config     = {false, 0, 0};
stream_config_t motor_torque_stream_config  = {false, 0, 0};

uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::CurrentPID> > current_pid_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::VelocityPID> > velocity_pid_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::PositionPID> > position_pid_pub;

uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::Index> > index_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> > enc_pos_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorPosition> > motor_pos_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorTorque> > motor_torque_pub;

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

int uavcan_streams_start(Node &node)
{
    int res;

    current_pid_pub.construct<Node &>(node);
    res = current_pid_pub->init();
    if (res < 0) {
        return res;
    }

    velocity_pid_pub.construct<Node &>(node);
    res = velocity_pid_pub->init();
    if (res < 0) {
        return res;
    }

    position_pid_pub.construct<Node &>(node);
    res = position_pid_pub->init();
    if (res < 0) {
        return res;
    }

    index_pub.construct<Node &>(node);
    res = index_pub->init();
    if (res < 0) {
        return res;
    }

    enc_pos_pub.construct<Node &>(node);
    res = enc_pos_pub->init();
    if (res < 0) {
        return res;
    }

    motor_pos_pub.construct<Node &>(node);
    res = motor_pos_pub->init();
    if (res < 0) {
        return res;
    }

    motor_torque_pub.construct<Node &>(node);
    res = motor_torque_pub->init();
    if (res < 0) {
        return res;
    }

    static uavcan::ServiceServer<cvra::motor::config::FeedbackStream> feedback_stream_sub(node);

    res = feedback_stream_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::config::FeedbackStream::Request>& req,
            cvra::motor::config::FeedbackStream::Response& rsp)
    {

        (void) rsp;

        switch (req.stream) {
            case cvra::motor::config::FeedbackStream::Request::STREAM_CURRENT_PID:
                stream_init_from_callback(&current_pid_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_VELOCITY_PID:
                stream_init_from_callback(&velocity_pid_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_POSITION_PID:
                stream_init_from_callback(&position_pid_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_INDEX:
                stream_init_from_callback(&index_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_ENCODER:
                stream_init_from_callback(&motor_enc_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_POSITION:
                stream_init_from_callback(&motor_pos_stream_config, req);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_TORQUE:
                stream_init_from_callback(&motor_torque_stream_config, req);
                break;
        }
    });

    if (res < 0) {
        return res;
    }

    return 0;
}

void uavcan_streams_spin(Node &node)
{
    /* Streams */
    if (stream_update(&current_pid_stream_config)) {
        cvra::motor::feedback::CurrentPID current_pid;
        current_pid.current = control_get_current();
        current_pid.current_setpoint = control_get_current_setpoint();
        current_pid.motor_voltage = control_get_motor_voltage();
        current_pid_pub->broadcast(current_pid);
    }

    if (stream_update(&velocity_pid_stream_config)) {
        cvra::motor::feedback::VelocityPID velocity_pid;
        velocity_pid.velocity = control_get_velocity();
        velocity_pid.velocity_setpoint = control_get_velocity_setpoint();
        velocity_pid_pub->broadcast(velocity_pid);
    }

    if (stream_update(&position_pid_stream_config)) {
        cvra::motor::feedback::PositionPID position_pid;
        position_pid.position = control_get_position();
        position_pid.position_setpoint = control_get_position_setpoint();
        position_pid_pub->broadcast(position_pid);
    }

    if (stream_update(&motor_enc_stream_config)) {
        cvra::motor::feedback::MotorEncoderPosition enc_pos;
        enc_pos.raw_encoder_position = encoder_get_secondary();
        enc_pos_pub->broadcast(enc_pos);
    }

    if (stream_update(&index_stream_config)) {
        cvra::motor::feedback::Index index;
        float index_pos;
        uint32_t update_count;
        index_get_position(&index_pos, &update_count);
        index.position = index_pos;
        index.update_count = update_count;
        index_pub->broadcast(index);
    }

    if (stream_update(&motor_pos_stream_config)) {
        cvra::motor::feedback::MotorPosition motor_pos;
        motor_pos.position = control_get_position();
        motor_pos.velocity = control_get_velocity();
        motor_pos_pub->broadcast(motor_pos);
    }

    if (stream_update(&motor_torque_stream_config)) {
        cvra::motor::feedback::MotorTorque motor_torque;
        motor_torque.torque = control_get_torque();
        motor_torque.position = control_get_position();
        motor_torque_pub->broadcast(motor_torque);
    }
}
