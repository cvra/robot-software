
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/config/LoadConfiguration.hpp>
#include <cvra/motor/config/FeedbackStream.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

#include <error/error.h>
#include <timestamp/timestamp.h>
#include "uavcan_node.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.hpp"
#include "main.h"

using namespace uavcan;
using namespace cvra::motor;

/** Sends the initial configuration to the given motor, loading the values from
 * the global parameter tree. */
static void motor_driver_send_initial_config(motor_driver_t *d);

/*** Sends a setpoint to the motor board, picking the message type according to
 * the current value. */
static void motor_driver_uavcan_send_setpoint(motor_driver_t *d);

/** Send new parameters from the global tree to the motor board. */
static int motor_driver_uavcan_update_config(motor_driver_t *d);

/** Logs an error in case the UAVCAN RPC failed.
 *
 * It is parametrized to adapt to any type of RPC. */
template<typename T>
static void assert_call_successful(const ServiceCallResult<T>& call_result);

static LazyConstructor<ServiceClient<config::VelocityPID> > speed_pid_client;
static LazyConstructor<ServiceClient<config::PositionPID> > position_pid_client;
static LazyConstructor<ServiceClient<config::CurrentPID> > current_pid_client;
static LazyConstructor<ServiceClient<config::LoadConfiguration> > config_client;

static LazyConstructor<ServiceClient<config::FeedbackStream> > feedback_stream_client;
static LazyConstructor<Publisher<control::Velocity> > velocity_pub;
static LazyConstructor<Publisher<control::Position> > position_pub;
static LazyConstructor<Publisher<control::Torque> > torque_pub;
static LazyConstructor<Publisher<control::Voltage> > voltage_pub;

int motor_driver_uavcan_init(INode &node)
{
    int res;

    speed_pid_client.construct<INode &>(node);
    res = speed_pid_client->init();
    if (res != 0) {
        return res;
    }
    speed_pid_client->setCallback(assert_call_successful<config::VelocityPID>);

    position_pid_client.construct<INode &>(node);
    res = position_pid_client->init();
    if (res != 0) {
        return res;
    }
    position_pid_client->setCallback(assert_call_successful<config::PositionPID>);

    current_pid_client.construct<INode &>(node);
    res = current_pid_client->init();
    if (res != 0) {
        return res;
    }
    current_pid_client->setCallback(assert_call_successful<config::CurrentPID>);

    config_client.construct<INode &>(node);
    res = config_client->init();
    if (res != 0) {
        return res;
    }
    config_client->setCallback(assert_call_successful<config::LoadConfiguration>);

    feedback_stream_client.construct<INode &>(node);
    res = feedback_stream_client->init();
    if (res != 0) {
        return res;
    }
    feedback_stream_client->setCallback(assert_call_successful<config::FeedbackStream>);

    velocity_pub.construct<INode &>(node);
    position_pub.construct<INode &>(node);
    torque_pub.construct<INode &>(node);
    voltage_pub.construct<INode &>(node);

    /* Setup a timer that will send the config & setpoints to the motor boards
     * periodically.
     *
     * This timer will be called from the UAVCAN main event loop.
     * */
    static Timer periodic_timer(node);
    periodic_timer.setCallback(
        [&](const TimerEvent &event)
    {
        (void) event;

        motor_driver_t *drv_list;
        uint16_t drv_list_len;

        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);

        for (int i = 0; i < drv_list_len; i++) {
            /* Only update one motor per 50 ms to avoid overloading the bus. */
            if (motor_driver_uavcan_update_config(&drv_list[i])) {
                break;
            }
        }

        for (int i = 0; i < drv_list_len; i++) {
            motor_driver_uavcan_send_setpoint(&drv_list[i]);
        }
    });

    /* Starts the periodic timer. Its rate must be at least every 300 ms,
     * otherwise the motor boards disable their output. Packet drop also needs
     * to be taken into account here. */
    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(50));

    return 0;
}

static void update_motor_can_id(motor_driver_t *d)
{
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        node_id = bus_enumerator_get_can_id(&bus_enumerator, motor_driver_get_id(d));
        if (node_id != BUS_ENUMERATOR_CAN_ID_NOT_SET
            && node_id != BUS_ENUMERATOR_STRING_ID_NOT_FOUND) {
            motor_driver_set_can_id(d, node_id);
        }
    }
}

static void send_stream_config(int node_id, uint8_t stream, float frequency)
{
    config::FeedbackStream::Request feedback_stream_config;

    feedback_stream_config.stream = stream;
    if (frequency == 0) {
        feedback_stream_config.enabled = false;
    } else {
        feedback_stream_config.enabled = true;
    }
    feedback_stream_config.frequency = frequency;

    feedback_stream_client->call(node_id, feedback_stream_config);
}

static void motor_driver_send_initial_config(motor_driver_t *d)
{
    config::LoadConfiguration::Request config_msg;

    update_motor_can_id(d);
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return;
    }

    config_msg.position_pid.kp = parameter_scalar_get(&d->config.position_pid.kp);
    config_msg.position_pid.ki = parameter_scalar_get(&d->config.position_pid.ki);
    config_msg.position_pid.kd = parameter_scalar_get(&d->config.position_pid.kd);
    config_msg.position_pid.ilimit = parameter_scalar_get(&d->config.position_pid.ilimit);
    config_msg.velocity_pid.kp = parameter_scalar_get(&d->config.velocity_pid.kp);
    config_msg.velocity_pid.ki = parameter_scalar_get(&d->config.velocity_pid.ki);
    config_msg.velocity_pid.kd = parameter_scalar_get(&d->config.velocity_pid.kd);
    config_msg.velocity_pid.ilimit = parameter_scalar_get(&d->config.velocity_pid.ilimit);
    config_msg.current_pid.kp = parameter_scalar_get(&d->config.current_pid.kp);
    config_msg.current_pid.ki = parameter_scalar_get(&d->config.current_pid.ki);
    config_msg.current_pid.kd = parameter_scalar_get(&d->config.current_pid.kd);
    config_msg.current_pid.ilimit = parameter_scalar_get(&d->config.current_pid.ilimit);

    config_msg.torque_limit = parameter_scalar_get(&d->config.torque_limit);
    config_msg.velocity_limit = parameter_scalar_get(&d->config.velocity_limit);
    config_msg.acceleration_limit = parameter_scalar_get(&d->config.acceleration_limit);
    config_msg.low_batt_th = parameter_scalar_get(&d->config.low_batt_th);

    config_msg.thermal_capacity = parameter_scalar_get(&d->config.thermal_capacity);
    config_msg.thermal_resistance = parameter_scalar_get(&d->config.thermal_resistance);
    config_msg.thermal_current_gain = parameter_scalar_get(&d->config.thermal_current_gain);
    config_msg.max_temperature = parameter_scalar_get(&d->config.max_temperature);

    config_msg.torque_constant = parameter_scalar_get(&d->config.torque_constant);
    config_msg.transmission_ratio_p = parameter_integer_get(&d->config.transmission_ratio_p);
    config_msg.transmission_ratio_q = parameter_integer_get(&d->config.transmission_ratio_q);
    config_msg.motor_encoder_steps_per_revolution = parameter_integer_get(
        &d->config.motor_encoder_steps_per_revolution);
    config_msg.second_encoder_steps_per_revolution = parameter_integer_get(
        &d->config.second_encoder_steps_per_revolution);
    config_msg.potentiometer_gain = parameter_scalar_get(&d->config.potentiometer_gain);

    config_msg.mode = parameter_integer_get(&d->config.mode); // todo !

    config_client->call(node_id, config_msg);
}

static int motor_driver_uavcan_update_config(motor_driver_t *d)
{
    int changed = 0;
    update_motor_can_id(d);
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return 0;
    }

    if (parameter_namespace_contains_changed(&d->config.root)) {
        changed = 1;
    }

    if (parameter_namespace_contains_changed(&d->config.control)) {
        if (parameter_namespace_contains_changed(&d->config.position_pid.root)) {
            config::PositionPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.position_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.position_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.position_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.position_pid.ilimit);

            position_pid_client->call(node_id, request);
        }

        if (parameter_namespace_contains_changed(&d->config.velocity_pid.root)) {
            config::VelocityPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.velocity_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.velocity_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.velocity_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.velocity_pid.ilimit);

            speed_pid_client->call(node_id, request);
        }

        if (parameter_namespace_contains_changed(&d->config.current_pid.root)) {
            config::CurrentPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.current_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.current_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.current_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.current_pid.ilimit);

            current_pid_client->call(node_id, request);
        }
    }
    if (parameter_namespace_contains_changed(&d->config.stream)) {
        if (parameter_changed(&d->config.current_pid_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_CURRENT_PID,
                               parameter_scalar_get(&d->config.current_pid_stream));
        }
        if (parameter_changed(&d->config.velocity_pid_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_VELOCITY_PID,
                               parameter_scalar_get(&d->config.velocity_pid_stream));
        }
        if (parameter_changed(&d->config.position_pid_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_POSITION_PID,
                               parameter_scalar_get(&d->config.position_pid_stream));
        }
        if (parameter_changed(&d->config.index_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_INDEX,
                               parameter_scalar_get(&d->config.index_stream));
        }
        if (parameter_changed(&d->config.encoder_pos_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_MOTOR_ENCODER,
                               parameter_scalar_get(&d->config.encoder_pos_stream));
        }
        if (parameter_changed(&d->config.motor_pos_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_MOTOR_POSITION,
                               parameter_scalar_get(&d->config.motor_pos_stream));
        }
        if (parameter_changed(&d->config.motor_torque_stream)) {
            send_stream_config(node_id,
                               config::FeedbackStream::Request::STREAM_MOTOR_TORQUE,
                               parameter_scalar_get(&d->config.motor_torque_stream));
        }
    }
    if (parameter_namespace_contains_changed(&d->config.root)) {
        // still some changed parameters: need to resend full config
        motor_driver_send_initial_config(d);
    }

    return changed;
}

static void motor_driver_uavcan_send_setpoint(motor_driver_t *d)
{
    control::Position position_setpoint;
    control::Velocity velocity_setpoint;
    control::Torque torque_setpoint;
    control::Voltage voltage_setpoint;

    update_motor_can_id(d);
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return;
    }

    motor_driver_lock(d);
    switch (d->control_mode) {
        case MOTOR_CONTROL_MODE_VELOCITY: {
            velocity_setpoint.velocity = motor_driver_get_velocity_setpt(d);
            velocity_setpoint.node_id = node_id;
            velocity_pub->broadcast(velocity_setpoint);
        } break;

        case MOTOR_CONTROL_MODE_POSITION: {
            position_setpoint.position = motor_driver_get_position_setpt(d);
            position_setpoint.node_id = node_id;
            position_pub->broadcast(position_setpoint);
        } break;

        case MOTOR_CONTROL_MODE_TORQUE: {
            torque_setpoint.torque = motor_driver_get_torque_setpt(d);
            torque_setpoint.node_id = node_id;
            torque_pub->broadcast(torque_setpoint);
        } break;

        case MOTOR_CONTROL_MODE_VOLTAGE: {
            voltage_setpoint.voltage = motor_driver_get_voltage_setpt(d);
            voltage_setpoint.node_id = node_id;
            voltage_pub->broadcast(voltage_setpoint);
        } break;

        /* Nothing to do, not sending any setpoint will disable the board. */
        case MOTOR_CONTROL_MODE_DISABLED:
            break;

        default:
            ERROR("Unknown control mode %d for board %d", d->control_mode, node_id);
            break;
    }
    motor_driver_unlock(d);
}

template<typename T>
static void assert_call_successful(const ServiceCallResult<T>& call_result)
{
    if (!call_result.isSuccessful()) {
        ERROR("uavcan service call timeout %d", call_result.getCallID().server_node_id.get());
    }
}
