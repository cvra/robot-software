#include "motor_feedback_streams_handler.hpp"
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include "motor_driver.h"
#include "bus_enumerator.h"

int motor_feedback_stream_handler_init(uavcan::INode &node, bus_enumerator_t *enumerator)
{
    int res;

    static uavcan::Subscriber<cvra::motor::feedback::CurrentPID> current_pid_sub(node);
    res = current_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT, msg.current);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT_SETPT, msg.current_setpoint);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_VOLTAGE, msg.motor_voltage);
        }
    });

    if (res != 0) {
        return res;
    }

    static uavcan::Subscriber<cvra::motor::feedback::VelocityPID> velocity_pid_sub(node);
    res = velocity_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);

        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY, msg.velocity);
            motor_driver_set_stream_value(driver,
                                          MOTOR_STREAM_VELOCITY_SETPT,
                                          msg.velocity_setpoint);
        }
    });

    if (res != 0) {
        return res;
    }

    static uavcan::Subscriber<cvra::motor::feedback::PositionPID> position_pid_sub(node);
    res = position_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
            motor_driver_set_stream_value(driver,
                                          MOTOR_STREAM_POSITION_SETPT,
                                          msg.position_setpoint);
        }
    });

    if (res != 0) {
        return res;
    }

    static uavcan::Subscriber<cvra::motor::feedback::Index> index_sub(node);
    res = index_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_INDEX, msg.position);
            driver->stream.value_stream_index_update_count = msg.update_count;
        }
    });

    if (res != 0) {
        return res;
    }


    static uavcan::Subscriber<cvra::motor::feedback::MotorPosition> motor_pos_sub(node);
    res = motor_pos_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY, msg.velocity);
        }
    });

    if (res != 0) {
        return res;
    }


    static uavcan::Subscriber<cvra::motor::feedback::MotorTorque> motor_torque_sub(node);
    res = motor_torque_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg)
    {
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_TORQUE, msg.torque);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
        }
    });

    if (res != 0) {
        return res;
    }

    return 0;
}
