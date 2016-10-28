#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/motor/EmergencyStop.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/Reboot.hpp>
#include <cvra/StringID.hpp>
#include <cvra/proximity_beacon/Signal.hpp>
#include <msgbus/messagebus.h>
#include "error/error.h"
#include "robot_parameters.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.h"
#include "config.h"
#include "uavcan_node_private.hpp"
#include "uavcan_node.h"
#include "priorities.h"
#include "main.h"
#include "robot_helpers/beacon_helpers.h"

#include <errno.h>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192


bus_enumerator_t bus_enumerator;


namespace uavcan_node
{

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg);
static void node_fail(const char *reason);


static constexpr int RxQueueSize = 64;
static constexpr std::uint32_t BitRate = 1000000;

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

uavcan::ISystemClock& getSystemClock()
{
    return uavcan_stm32::SystemClock::instance();
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_stm32::CanInitHelper<RxQueueSize> can;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        int res = can.init(BitRate);
        if (res < 0) {
            node_fail("CAN driver");
        }
    }
    return can.driver;
}

Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

THD_WORKING_AREA(thread_wa, UAVCAN_NODE_STACK_SIZE);

void main(void *arg)
{
    chRegSetThreadName("uavcan");

    Node& node = getNode();

    uint8_t id = *(uint8_t *)arg;
    node.setNodeID(uavcan::NodeID(id));

    node.setName("cvra.master");

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    int res;
    res = node.start();
    if (res < 0) {
        node_fail("node start");
    }

    /*
     * NodeStatus subscriber
     */
    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
    res = ns_sub.start(node_status_cb);
    if (res < 0) {
        node_fail("NodeStatus subscribe");
    }

    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    res = emergency_stop_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
        {
            (void)msg;
            NVIC_SystemReset();
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::EmergencyStop subscriber");
    }

    uavcan::Subscriber<cvra::StringID> string_id_sub(node);
    res = string_id_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::StringID>& msg)
        {
            uint8_t can_id = msg.getSrcNodeID().get();

            if (bus_enumerator_get_str_id(&bus_enumerator, can_id) == NULL) {
                bus_enumerator_update_node_info(&bus_enumerator, msg.id.c_str(), can_id);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::StringID subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::CurrentPID> current_pid_sub(node);
    res = current_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::CurrentPID>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT, msg.current);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_CURRENT_SETPT, msg.current_setpoint);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_VOLTAGE, msg.motor_voltage);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::CurrentPID subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::VelocityPID> velocity_pid_sub(node);
    res = velocity_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::VelocityPID>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY, msg.velocity);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY_SETPT, msg.velocity_setpoint);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::VelocityPID subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::PositionPID> position_pid_sub(node);
    res = position_pid_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::PositionPID>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION_SETPT, msg.position_setpoint);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::PositionPID subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::Index> index_sub(node);
    res = index_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::Index>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_INDEX, msg.position);
                driver->stream.value_stream_index_update_count = msg.update_count;
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::Index subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::MotorPosition> motor_pos_sub(node);
    res = motor_pos_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorPosition>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_VELOCITY, msg.velocity);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::MotorPosition subscriber");
    }

    uavcan::Subscriber<cvra::motor::feedback::MotorTorque> motor_torque_sub(node);
    res = motor_torque_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorTorque>& msg)
        {
            motor_driver_t *driver = (motor_driver_t*)bus_enumerator_get_driver_by_can_id(&bus_enumerator, msg.getSrcNodeID().get());
            if (driver != NULL) {
                motor_driver_set_stream_value(driver, MOTOR_STREAM_MOTOR_TORQUE, msg.torque);
                motor_driver_set_stream_value(driver, MOTOR_STREAM_POSITION, msg.position);
            }
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::MotorTorque subscriber");
    }

    static messagebus_topic_t proximity_beacon_topic;
    static MUTEX_DECL(proximity_beacon_topic_lock);
    static CONDVAR_DECL(proximity_beacon_topic_condvar);
    static float proximity_beacon_topic_value[2];

    messagebus_topic_init(&proximity_beacon_topic,
                          &proximity_beacon_topic_lock,
                          &proximity_beacon_topic_condvar,
                          &proximity_beacon_topic_value,
                          sizeof(proximity_beacon_topic_value));

    messagebus_advertise_topic(&bus, &proximity_beacon_topic, "/proximity_beacon");
    const float reflector_diameter = 0.080f;
    const float angular_offset = M_PI / 2.;

    uavcan::Subscriber<cvra::proximity_beacon::Signal> prox_beac_sub(node);
    res = prox_beac_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::proximity_beacon::Signal>& msg)
        {
            float data[2];
            data[0] = reflector_diameter / (2. * sinf(msg.length / 2.));
            data[1] = beacon_get_angle(msg.start_angle + angular_offset, msg.length);
            messagebus_topic_publish(&proximity_beacon_topic, &data, sizeof(data));

            NOTICE("Opponent detected at: %.3fm, %.3frad \traw signal: %.3f, %.3f", data[0], data[1], msg.start_angle, msg.length);
        }
    );
    if (res < 0) {
        node_fail("cvra::proximity_beacon::Signal subscriber");
    }

    // Mark the node as correctly initialized
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();


    uavcan::Publisher<cvra::Reboot> reboot_pub(node);
    const int reboot_pub_init_res = reboot_pub.init();
    if (reboot_pub_init_res < 0) {
        node_fail("cvra::Reboot publisher");
    }

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
        if (res < 0) {
            // log warning
        }

        // reboot command
        if (board_button_pressed()) {
            cvra::Reboot reboot_msg;
            reboot_msg.bootmode = reboot_msg.BOOTLOADER_TIMEOUT;
            reboot_pub.broadcast(reboot_msg);
        }

        motor_driver_t *drv_list;
        uint16_t drv_list_len;
        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);
        int i;
        for (i = 0; i < drv_list_len; i++) {
            motor_driver_uavcan_update_config(&drv_list[i]);
            motor_driver_uavcan_send_setpoint(&drv_list[i]);
        }
    }
}

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
{
    if (msg.health != uavcan::protocol::NodeStatus::HEALTH_OK) {
        WARNING("UAVCAN node %u health", msg.getSrcNodeID().get());
    }
}

static void node_fail(const char *reason)
{
    (void) reason;
    ERROR("UAVCAN error: %s", reason);
    chSysHalt(reason);
}

} // namespace uavcan_node

extern "C" {

void uavcan_node_start(uint8_t id)
{
    static uint8_t node_id = id;
    chThdCreateStatic(uavcan_node::thread_wa, UAVCAN_NODE_STACK_SIZE, UAVCAN_PRIO, uavcan_node::main, &node_id);
}

} // extern "C"
