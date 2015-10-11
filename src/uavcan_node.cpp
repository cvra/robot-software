#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <lwip/api.h>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
#include <cvra/Reboot.hpp>
#include <cvra/StringID.hpp>
#include "robot_pose.h"
#include <simplerpc/message.h>
#include "src/rpc_server.h"
#include "robot_parameters.h"
#include "timestamp/timestamp.h"
#include "odometry/robot_base.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.h"
#include "odometry/odometry.h"
#include "config.h"
#include "uavcan_node_private.hpp"
#include "uavcan_node.h"
#include "node_tracker.h"
#include "main.h"

#include <errno.h>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192


bus_enumerator_t bus_enumerator;


namespace uavcan_node
{

uint8_t reboot_node_id = 0;

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
     * Initialising odometry
     */
    static odometry_differential_base_t robot_base;
    struct robot_base_pose_2d_s init_pose = {0.0f, 0.0f, 0.0f};
    odometry_base_init(&robot_base,
                       init_pose,
                       config_get_scalar("/master/odometry/radius_right"),
                       config_get_scalar("/master/odometry/radius_left"),
                       ROBOT_RIGHT_WHEEL_DIRECTION,
                       ROBOT_LEFT_WHEEL_DIRECTION,
                       config_get_scalar("/master/odometry/wheelbase"),
                       timestamp_get());

    static odometry_encoder_sample_t enc_right[2];
    static odometry_encoder_sample_t enc_left[2];
    odometry_encoder_record_sample(&enc_right[0], 0, 0);
    odometry_encoder_record_sample(&enc_right[1], 0, 0);
    odometry_encoder_record_sample(&enc_left[0], 0, 0);
    odometry_encoder_record_sample(&enc_left[1], 0, 0);

    /*
     * NodeStatus subscriber
     */
    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
    res = ns_sub.start(node_status_cb);
    if (res < 0) {
        node_fail("NodeStatus subscribe");
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

    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition> enc_pos_sub(node);
    res = enc_pos_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
        {
            if(msg.getSrcNodeID() == uavcan::NodeID(bus_enumerator_get_can_id(&bus_enumerator, "right-wheel"))) {
                odometry_encoder_record_sample(&enc_right[0], enc_right[1].timestamp, enc_right[1].value);
                odometry_encoder_record_sample(&enc_right[1], timestamp_get(), msg.raw_encoder_position);
                odometry_base_update(&robot_base, enc_right[1], enc_left[1]);   /* TODO shouldn't be called in here */
            } else if(msg.getSrcNodeID() == uavcan::NodeID(bus_enumerator_get_can_id(&bus_enumerator, "left-wheel"))) {
                odometry_encoder_record_sample(&enc_left[0], enc_left[1].timestamp, enc_left[1].value);
                odometry_encoder_record_sample(&enc_left[1], timestamp_get(), msg.raw_encoder_position);
            }

            parameter_namespace_t *odometry_ns;
            odometry_ns = parameter_namespace_find(&global_config, "/master/odometry");
            if (parameter_namespace_contains_changed(odometry_ns)) {
                odometry_base_set_parameters(&robot_base,
                                             config_get_scalar("/master/odometry/wheelbase"),
                                             config_get_scalar("/master/odometry/radius_right"),
                                             config_get_scalar("/master/odometry/radius_left"));

            }

            /* update global robot pose */
            chMtxLock(&robot_pose_lock);
            odometry_base_get_pose(&robot_base, &robot_pose);
            chMtxUnlock(&robot_pose_lock);
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::MotorEncoderPosition subscriber");
    }

    // Mark the node as correctly initialized
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();


    uavcan::Publisher<cvra::Reboot> reboot_pub(node);
    const int reboot_pub_init_res = reboot_pub.init();
    if (reboot_pub_init_res < 0)
    {
        node_fail("cvra::Reboot publisher");
    }

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
        if (res < 0) {
            // log warning
        }

        // reboot command
        int button = palReadPad(GPIOA, GPIOA_BUTTON_WKUP);
        if (button) {
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

        // todo: publish time once a second
        // time_sync_master.publish();
    }
}

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
{
    (void) msg;
    // TODO !!!
    // int can_id = msg.getSrcNodeID().get();
    // bus_enumerator_get_driver()
    // motor_driver_send_initial_config()
    node_tracker_set_id((uint8_t)msg.getSrcNodeID().get());
}

static void node_fail(const char *reason)
{
    (void) reason;
    while (1) {
        chThdSleepMilliseconds(100);
    }
}

} // namespace uavcan_node

extern "C" {

void uavcan_node_start(uint8_t id)
{
    static uint8_t node_id = id;
    chThdCreateStatic(uavcan_node::thread_wa, UAVCAN_NODE_STACK_SIZE, NORMALPRIO, uavcan_node::main, &node_id);
}

void uavcan_node_send_reboot(uint8_t id)
{
    uavcan_node::reboot_node_id = id;
}

} // extern "C"
