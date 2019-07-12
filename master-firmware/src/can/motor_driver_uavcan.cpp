#include <ch.h>
#include <hal.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/param/GetSet.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include <cvra/motor/control/Torque.hpp>
#include <cvra/motor/control/Voltage.hpp>

#include <error/error.h>
#include <timestamp/timestamp.h>
#include <uavcan_node.h>
#include "motor_driver.h"
#include "motor_driver_uavcan.hpp"
#include "control_panel.h"
#include "main.h"

using namespace uavcan;
using namespace cvra::motor;

/*** Sends a setpoint to the motor board, picking the message type according to
 * the current value. */
static void motor_driver_uavcan_send_setpoint(motor_driver_t* d);

/** Send new parameters from the global tree to the motor board. */
static int motor_driver_uavcan_update_config(motor_driver_t* d);

/** Logs an error in case the UAVCAN RPC failed.
 *
 * It is parametrized to adapt to any type of RPC. */
template <typename T>
static void assert_call_successful(const ServiceCallResult<T>& call_result);

static LazyConstructor<ServiceClient<uavcan::protocol::param::GetSet>> feedback_stream_client;
static LazyConstructor<Publisher<control::Velocity>> velocity_pub;
static LazyConstructor<Publisher<control::Position>> position_pub;
static LazyConstructor<Publisher<control::Torque>> torque_pub;
static LazyConstructor<Publisher<control::Voltage>> voltage_pub;

int motor_driver_uavcan_init(INode& node)
{
    int res;

    feedback_stream_client.construct<INode&>(node);
    res = feedback_stream_client->init();
    if (res != 0) {
        return res;
    }
    feedback_stream_client->setCallback(assert_call_successful<uavcan::protocol::param::GetSet>);

    velocity_pub.construct<INode&>(node);
    position_pub.construct<INode&>(node);
    torque_pub.construct<INode&>(node);
    voltage_pub.construct<INode&>(node);

    /* Setup a timer that will send the config & setpoints to the motor boards
     * periodically.
     *
     * This timer will be called from the UAVCAN main event loop.
     * */
    static Timer periodic_timer(node);
    periodic_timer.setCallback(
        [&](const TimerEvent& event) {
            (void)event;

            motor_driver_t* drv_list;
            uint16_t drv_list_len;

            motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);

            control_panel_clear(LED_BUS);

            for (int i = 0; i < drv_list_len; i++) {
                /* Only update one motor per 50 ms to avoid overloading the bus. */
                if (motor_driver_uavcan_update_config(&drv_list[i])) {
                    control_panel_set(LED_BUS);
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

static void update_motor_can_id(motor_driver_t* d)
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

static void send_stream_config(int node_id, float frequency, const char* request_name)
{
    uavcan::protocol::param::GetSet::Request request;

    request.name = request_name;
    request.value.to<uavcan::protocol::param::Value::Tag::real_value>() = frequency;

    feedback_stream_client->call(node_id, request);
}

static int motor_driver_uavcan_update_config(motor_driver_t* d)
{
    update_motor_can_id(d);
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return 0;
    }

    return 1;
}

static void motor_driver_uavcan_send_setpoint(motor_driver_t* d)
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

template <typename T>
static void assert_call_successful(const ServiceCallResult<T>& call_result)
{
    if (!call_result.isSuccessful()) {
        ERROR("uavcan service call timeout %d", call_result.getCallID().server_node_id.get());
    }
}
