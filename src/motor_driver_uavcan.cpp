
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/motor/config/SpeedPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/motor/control/Position.hpp>
#include "motor_driver.h"
#include "motor_driver_uavcan.h"
#include "uavcan_node_private.hpp"

struct can_driver_s {
    uavcan::ServiceClient<cvra::motor::config::SpeedPID> speed_pid_client;
    uavcan::ServiceClient<cvra::motor::config::PositionPID> position_pid_client;
    uavcan::ServiceClient<cvra::motor::config::CurrentPID> current_pid_client;
    uavcan::Publisher<cvra::motor::control::Velocity> velocity_pub;
    uavcan::Publisher<cvra::motor::control::Position> position_pub;

    can_driver_s():
        speed_pid_client(getNode()),
        position_pid_client(getNode()),
        current_pid_client(getNode()),
        velocity_pub(getNode()),
        position_pub(getNode())
    {

    }
};


static void driver_allocation(motor_driver_t *d)
{
    if (d->can_driver == NULL) {
        d->can_driver = new struct can_driver_s;
        if (d->can_driver == NULL) {
            chSysHalt("motor driver memory allocation failed");
        }
    }
}


void motor_driver_uavcan_update_config(motor_driver_t *d)
{
    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return;
    }
    driver_allocation(d);
    struct can_driver_s *can_drv = (struct can_driver_s*)d->can_driver;
    if (parameter_namespace_contains_changed(&d->config.pid_root)) {
        if (parameter_namespace_contains_changed(&d->config.position_pid.root)) {
            cvra::motor::config::PositionPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.position_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.position_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.position_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.position_pid.ilimit);
            can_drv->position_pid_client.call(node_id, request);
        }

        if (parameter_namespace_contains_changed(&d->config.velocity_pid.root)) {
            cvra::motor::config::SpeedPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.velocity_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.velocity_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.velocity_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.velocity_pid.ilimit);
            can_drv->speed_pid_client.call(node_id, request);
        }

        if (parameter_namespace_contains_changed(&d->config.current_pid.root)) {
            cvra::motor::config::CurrentPID::Request request;
            request.pid.kp = parameter_scalar_get(&d->config.current_pid.kp);
            request.pid.ki = parameter_scalar_get(&d->config.current_pid.ki);
            request.pid.kd = parameter_scalar_get(&d->config.current_pid.kd);
            request.pid.ilimit = parameter_scalar_get(&d->config.current_pid.ilimit);
            can_drv->current_pid_client.call(node_id, request);
        }
    }
}

void motor_driver_uavcan_send_setpoint(motor_driver_t *d)
{
    cvra::motor::control::Position position_setpoint;
    cvra::motor::control::Velocity velocity_setpoint;

    int node_id = motor_driver_get_can_id(d);
    if (node_id == CAN_ID_NOT_SET) {
        return;
    }
    driver_allocation(d);
    can_driver_s *can_drv = (can_driver_s*)d->can_driver;

    switch(d->control_mode) {
        case MOTOR_CONTROL_MODE_VELOCITY:
            velocity_setpoint.velocity = d->setpt.velocity;
            can_drv->velocity_pub.unicast(velocity_setpoint, node_id);
            break;

        case MOTOR_CONTROL_MODE_POSITION:
            position_setpoint.position = d->setpt.position;
            can_drv->position_pub.unicast(position_setpoint, node_id);
            break;

        case MOTOR_CONTROL_MODE_TORQUE:
        case MOTOR_CONTROL_MODE_VOLTAGE:
        case MOTOR_CONTROL_MODE_TRAJECTORY:
            /* TODO */
        case MOTOR_CONTROL_MODE_DISABLED:
        default:
            /* TODO */
            break;



            break;

    }
}
