#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <lwip/api.h>
#include <src/can_bridge.h>
#include <cvra/Reboot.hpp>
#include <cvra/motor/control/Velocity.hpp>
#include <cvra/Reboot.hpp>
#include <cvra/motor/config/VelocityPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include <cvra/motor/feedback/CurrentPID.hpp>
#include <cvra/motor/feedback/VelocityPID.hpp>
#include <cvra/motor/feedback/PositionPID.hpp>
#include <cvra/motor/feedback/Index.hpp>
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/feedback/MotorPosition.hpp>
#include <cvra/motor/feedback/MotorTorque.hpp>
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

#include <uavcan/protocol/global_time_sync_master.hpp>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192


bus_enumerator_t bus_enumerator;


namespace uavcan_node
{

uavcan_stm32::CanInitHelper<128> can;

uavcan::LazyConstructor<Node> node_;

uint8_t reboot_node_id = 0;

Node& getNode()
{
    if (!node_.isConstructed())
    {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
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

void can_bridge_send_frames(Node& node)
{
    while (1) {
        struct can_frame *framep;
        msg_t m = chMBFetch(&can_bridge_tx_queue, (msg_t *)&framep, TIME_IMMEDIATE);
        if (m == MSG_OK) {
            uint32_t id = framep->id;
            uavcan::CanFrame ucframe;
            if (id & CAN_FRAME_EXT_FLAG) {
                ucframe.id = id & CAN_FRAME_EXT_ID_MASK;
                ucframe.id |= uavcan::CanFrame::FlagEFF;
            } else {
                ucframe.id = id & CAN_FRAME_STD_ID_MASK;
            }

            if (id & CAN_FRAME_RTR_FLAG) {
                ucframe.id |= uavcan::CanFrame::FlagRTR;
            }

            ucframe.dlc = framep->dlc;
            memcpy(ucframe.data, framep->data.u8, framep->dlc);

            if (can.driver.wait_tx_mb0(MS2ST(100))) {
                uavcan::MonotonicTime tx_timeout = node.getMonotonicTime();
                tx_timeout += uavcan::MonotonicDuration::fromMSec(100);
                uavcan::ICanIface* const iface = can.driver.getIface(0);
                iface->send(ucframe, tx_timeout, 0);
            }
            chPoolFree(&can_bridge_tx_pool, framep);
        } else {
            break;
        }
    }
}

THD_WORKING_AREA(thread_wa, UAVCAN_NODE_STACK_SIZE);

void main(void *arg)
{
    chRegSetThreadName("uavcan");

    // bridge has to be initialized first
    chSemWait(&can_bridge_is_initialized);

    uint8_t id = *(uint8_t *)arg;

    int res;
    res = can.init(1000000);
    if (res < 0) {
        node_fail("CAN init");
    }

    Node& node = getNode();

    node.setNodeID(id);
    node.setName("cvra.master");

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
     * Initializing the UAVCAN node - this may take a while
     */
    while (true) {
        // Calling start() multiple times is OK - only the first successfull call will be effective
        int res = node.start();

        uavcan::NetworkCompatibilityCheckResult ncc_result;

        if (res >= 0) {
            res = node.checkNetworkCompatibility(ncc_result);
            if (res >= 0) {
                break;
            }
        }
        chThdSleepMilliseconds(1000);
    }

    /*
     * Time synchronizer
     */
    uavcan::UtcDuration adjustment;
    uint64_t utc_time_init = 1234;
    adjustment = uavcan::UtcTime::fromUSec(utc_time_init) - uavcan::UtcTime::fromUSec(0);
    node.getSystemClock().adjustUtc(adjustment);

    static uavcan::GlobalTimeSyncMaster time_sync_master(node);
    res = time_sync_master.init();;
    if (res < 0) {
        node_fail("TimeSyncMaster");
    }

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
            chMtxLock(&robot_pose_lock);
            if(msg.getSrcNodeID().get() == 29) {
                odometry_encoder_record_sample(&enc_right[0], enc_right[1].timestamp, enc_right[1].value);
                odometry_encoder_record_sample(&enc_right[1], timestamp_get(), msg.raw_encoder_position);
                palTogglePad(GPIOF, GPIOF_LED_YELLOW_1);
            } else if(msg.getSrcNodeID().get() == 31) {
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

                palTogglePad(GPIOF, GPIOF_LED_YELLOW_2);
            }

            /*
             * Only call odometry update when an encoder value has been registered for each wheel
             * ie when the last timestamp recorded is different from the previous one
             */
            if(enc_right[1].timestamp != enc_right[0].timestamp && enc_left[1].timestamp != enc_left[0].timestamp) {
                odometry_base_update(&robot_base, enc_right[1], enc_left[1]);
                odometry_base_get_pose(&robot_base, &robot_pose);
            }
            chMtxUnlock(&robot_pose_lock);
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::feedback::MotorEncoderPosition subscriber");
    }

    node.setStatusOk();


    uavcan::Publisher<cvra::Reboot> reboot_pub(node);
    const int reboot_pub_init_res = reboot_pub.init();
    if (reboot_pub_init_res < 0)
    {
        node_fail("cvra::Reboot publisher");
    }

    uavcan::Publisher<cvra::motor::control::Velocity> velocity_ctrl_setpt_pub(node);
    const int velocity_ctrl_setpt_pub_init_res = velocity_ctrl_setpt_pub.init();
    if (velocity_ctrl_setpt_pub_init_res < 0)
    {
        node_fail("cvra::motor::control::Velocity publisher");
    }


    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
        if (res < 0) {
            // log warning
        }

        // reboot command
        int button = palReadPad(GPIOA, GPIOA_BUTTON_WKUP);
        if (button || reboot_node_id) {
            cvra::Reboot reboot_msg;
            reboot_msg.bootmode = reboot_msg.BOOTLOADER_TIMEOUT;
            if (button || reboot_node_id > 127) {
                reboot_pub.broadcast(reboot_msg);
            } else {

#warning "Unicast is simply disabled. Won't work as is."
//                reboot_pub.unicast(reboot_msg, uavcan::NodeID(reboot_node_id));
            }
            reboot_node_id = 0;
        }

        motor_driver_t *drv_list;
        uint16_t drv_list_len;
        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);
        int i;
        for (i = 0; i < drv_list_len; i++) {
            motor_driver_uavcan_update_config(&drv_list[i]);
            motor_driver_uavcan_send_setpoint(&drv_list[i]);
        }

        can_bridge_send_frames(node);

        // todo: publish time once a second
        // time_sync_master.publish();
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
