#include <parameter/parameter.h>
#include "uavcan_streams.hpp"
#include "stream.h"

#include "control.h"
#include "index.h"
#include "encoder.h"
#include "main.h"

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
stream_config_t enc_pos_stream_config       = {false, 0, 0};
stream_config_t motor_pos_stream_config     = {false, 0, 0};
stream_config_t motor_torque_stream_config  = {false, 0, 0};

uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::CurrentPID> > current_pid_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::VelocityPID> > velocity_pid_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::PositionPID> > position_pid_pub;

uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::Index> > index_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorEncoderPosition> > enc_pos_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorPosition> > motor_pos_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::motor::feedback::MotorTorque> > motor_torque_pub;

static struct {
    parameter_namespace_t ns;
    parameter_t current, velocity, position, index, enc_pos, motor_pos, motor_torque;
} stream_params;

static void stream_update_from_parameters(stream_config_t *conf, parameter_t *freq_param)
{
    const float frequency = parameter_scalar_get(freq_param);

    if (frequency > 0.f) {
        stream_set_prescaler(conf, frequency, UAVCAN_SPIN_FREQUENCY);
        stream_enable(conf, true);
    } else {
        stream_enable(conf, false);
    }
}

int uavcan_streams_start(Node &node)
{
    int res;

    parameter_namespace_declare(&stream_params.ns, &parameter_root_ns, "streams");
    parameter_scalar_declare_with_default(&stream_params.current, &stream_params.ns, "current", 0);
    parameter_scalar_declare_with_default(&stream_params.velocity, &stream_params.ns, "velocity", 0);
    parameter_scalar_declare_with_default(&stream_params.position, &stream_params.ns, "position", 0);
    parameter_scalar_declare_with_default(&stream_params.index, &stream_params.ns, "index", 0);
    parameter_scalar_declare_with_default(&stream_params.enc_pos, &stream_params.ns, "enc_pos", 0);
    parameter_scalar_declare_with_default(&stream_params.motor_pos, &stream_params.ns, "motor_pos", 0);
    parameter_scalar_declare_with_default(&stream_params.motor_torque, &stream_params.ns, "motor_torque", 0);

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
                parameter_scalar_set(&stream_params.current, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_VELOCITY_PID:
                parameter_scalar_set(&stream_params.velocity, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_POSITION_PID:
                parameter_scalar_set(&stream_params.position, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_INDEX:
                parameter_scalar_set(&stream_params.index, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_ENCODER:
                parameter_scalar_set(&stream_params.enc_pos, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_POSITION:
                parameter_scalar_set(&stream_params.motor_pos, req.frequency);
                break;

            case cvra::motor::config::FeedbackStream::Request::STREAM_MOTOR_TORQUE:
                parameter_scalar_set(&stream_params.motor_torque, req.frequency);
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
    (void) node;

    if (parameter_namespace_contains_changed(&stream_params.ns)) {
        stream_update_from_parameters(&current_pid_stream_config, &stream_params.current);
        stream_update_from_parameters(&velocity_pid_stream_config, &stream_params.velocity);
        stream_update_from_parameters(&position_pid_stream_config, &stream_params.position);
        stream_update_from_parameters(&index_stream_config, &stream_params.index);
        stream_update_from_parameters(&enc_pos_stream_config, &stream_params.enc_pos);
        stream_update_from_parameters(&motor_pos_stream_config, &stream_params.motor_pos);
        stream_update_from_parameters(&motor_torque_stream_config, &stream_params.motor_torque);
    }

    /* Streams */
    if (stream_should_send(&current_pid_stream_config)) {
        cvra::motor::feedback::CurrentPID current_pid;
        current_pid.current = control_get_current();
        current_pid.current_setpoint = control_get_current_setpoint();
        current_pid.motor_voltage = control_get_motor_voltage();
        current_pid_pub->broadcast(current_pid);
    }

    if (stream_should_send(&velocity_pid_stream_config)) {
        cvra::motor::feedback::VelocityPID velocity_pid;
        velocity_pid.velocity = control_get_velocity();
        velocity_pid.velocity_setpoint = control_get_velocity_setpoint();
        velocity_pid_pub->broadcast(velocity_pid);
    }

    if (stream_should_send(&position_pid_stream_config)) {
        cvra::motor::feedback::PositionPID position_pid;
        position_pid.position = control_get_position();
        position_pid.position_setpoint = control_get_position_setpoint();
        position_pid_pub->broadcast(position_pid);
    }

    if (stream_should_send(&enc_pos_stream_config)) {
        cvra::motor::feedback::MotorEncoderPosition enc_pos;
        enc_pos.raw_encoder_position = encoder_get_secondary();
        enc_pos_pub->broadcast(enc_pos);
    }

    if (stream_should_send(&index_stream_config)) {
        cvra::motor::feedback::Index index;
        float index_pos;
        uint32_t update_count;
        index_get_position(&index_pos, &update_count);
        index.position = index_pos;
        index.update_count = update_count;
        index_pub->broadcast(index);
    }

    if (stream_should_send(&motor_pos_stream_config)) {
        cvra::motor::feedback::MotorPosition motor_pos;
        motor_pos.position = control_get_position();
        motor_pos.velocity = control_get_velocity();
        motor_pos_pub->broadcast(motor_pos);
    }

    if (stream_should_send(&motor_torque_stream_config)) {
        cvra::motor::feedback::MotorTorque motor_torque;
        motor_torque.torque = control_get_torque();
        motor_torque.position = control_get_position();
        motor_torque_pub->broadcast(motor_torque);
    }
}
