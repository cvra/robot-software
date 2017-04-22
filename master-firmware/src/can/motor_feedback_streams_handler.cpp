#include "motor_feedback_streams_handler.hpp"
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include "motor_driver.h"
#include "bus_enumerator.h"

using namespace uavcan;
using namespace cvra::motor;

static bus_enumerator_t *enumerator;

static void current_pid_cb(const ReceivedDataStructure<feedback::CurrentPID>& msg)
{
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT, msg.current);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT_SETPT, msg.current_setpoint);
            motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_VOLTAGE, msg.motor_voltage);
        }
}

static void velocity_pid_cb(const ReceivedDataStructure<feedback::VelocityPID>& msg)
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
}

static void position_pid_cb(const ReceivedDataStructure<feedback::PositionPID>& msg)
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
}

static void index_cb(const ReceivedDataStructure<feedback::Index>& msg)
{
        int id = msg.getSrcNodeID().get();
        motor_driver_t *driver;
        driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
        if (driver != NULL) {
            motor_driver_set_stream_value(driver, MOTOR_STREAM_INDEX, msg.position);
            driver->stream.value_stream_index_update_count = msg.update_count;
        }
}

static void motor_pos_cb(const ReceivedDataStructure<feedback::MotorPosition>& msg)
{
    int id = msg.getSrcNodeID().get();
    motor_driver_t *driver;
    driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
    if (driver != NULL) {
        motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
        motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY, msg.velocity);
    }
}

static void torque_cb(const ReceivedDataStructure<feedback::MotorTorque>& msg)
{
    int id = msg.getSrcNodeID().get();
    motor_driver_t *driver;
    driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(enumerator, id);
    if (driver != NULL) {
        motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_TORQUE, msg.torque);
        motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
    }
}

int motor_feedback_stream_handler_init(INode &node, bus_enumerator_t *e)
{
    int res;

    enumerator = e;

    static Subscriber<feedback::CurrentPID> current_pid_sub(node);
    res = current_pid_sub.start(current_pid_cb);

    if (res != 0) {
        return res;
    }

    static Subscriber<feedback::VelocityPID> velocity_pid_sub(node);
    res = velocity_pid_sub.start(velocity_pid_cb);

    if (res != 0) {
        return res;
    }

    static Subscriber<feedback::PositionPID> position_pid_sub(node);
    res = position_pid_sub.start(position_pid_cb);

    if (res != 0) {
        return res;
    }

    static Subscriber<feedback::Index> index_sub(node);
    res = index_sub.start(index_cb);

    if (res != 0) {
        return res;
    }


    static Subscriber<feedback::MotorPosition> motor_pos_sub(node);
    res = motor_pos_sub.start(motor_pos_cb);

    if (res != 0) {
        return res;
    }


    static Subscriber<feedback::MotorTorque> motor_torque_sub(node);
    res = motor_torque_sub.start(torque_cb);

    if (res != 0) {
        return res;
    }
    return 0;
}
