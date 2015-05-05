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
#include <cvra/motor/feedback/MotorEncoderPosition.hpp>
#include <cvra/motor/config/SpeedPID.hpp>
#include <cvra/motor/config/PositionPID.hpp>
#include <cvra/motor/config/CurrentPID.hpp>
#include "motor_control.h"
#include <simplerpc/message.h>
#include "src/rpc_server.h"
#include "robot_parameters.h"
#include "timestamp/timestamp.h"
#include "odometry/robot_base.h"
#include "odometry/odometry.h"
#include "config.h"

#include <errno.h>

#include <uavcan/protocol/global_time_sync_master.hpp>

#define RIGHT_WHEEL_ID  11
#define LEFT_WHEEL_ID   10

#define UAVCAN_NODE_STACK_SIZE 4096

namespace uavcan_node
{

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<16384> Node;

uavcan::LazyConstructor<Node> node_;

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

msg_t main(void *arg)
{
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


    uavcan::Subscriber<cvra::motor::feedback::MotorEncoderPosition> enc_pos_sub(node);
    res = enc_pos_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::feedback::MotorEncoderPosition>& msg)
        {
            static uint8_t buffer[512];
            cmp_ctx_t ctx;
            cmp_mem_access_t mem;
            ip_addr_t server;
            IP4_ADDR(&server, 192, 168, 2, 1);

            if(msg.getSrcNodeID() == RIGHT_WHEEL_ID) {
                odometry_encoder_record_sample(&enc_right[0], enc_right[1].timestamp, enc_right[1].value);
                odometry_encoder_record_sample(&enc_right[1], timestamp_get(), msg.raw_encoder_position);
            } else if(msg.getSrcNodeID() == LEFT_WHEEL_ID) {
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

            /*
             * Only call odometry update when an encoder value has been registered for each wheel
             * ie when the last timestamp recorded is different from the previous one
             */
            if(enc_right[1].timestamp != enc_right[0].timestamp && enc_left[1].timestamp != enc_left[0].timestamp) {
                odometry_base_update(&robot_base, enc_right[1], enc_left[1]);
            }

            struct robot_base_pose_2d_s pose;
            odometry_base_get_pose(&robot_base, &pose);

            message_encode(&ctx, &mem, buffer, sizeof buffer, "odometry_raw", 3);
            cmp_write_float(&ctx, pose.x);
            cmp_write_float(&ctx, pose.y);
            cmp_write_float(&ctx, pose.theta);
            message_transmit(buffer, cmp_mem_access_get_pos(&mem), &server, 20000);

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

    /* Config client. */
    uavcan::ServiceClient<cvra::motor::config::SpeedPID> speed_pid_client(node);
    if (speed_pid_client.init() < 0) {
        node_fail("cvra::motor::config::SpeedPID client");
    }

    speed_pid_client.setCallback([](const uavcan::ServiceCallResult<cvra::motor::config::SpeedPID>& call_result)
    {
        if (call_result.isSuccessful() == false) {
            // TODO: error handling.
        }
    });

    uavcan::ServiceClient<cvra::motor::config::PositionPID> position_pid_client(node);
    if (position_pid_client.init() < 0) {
        node_fail("cvra::motor::config::PositionPID client");
    }

    position_pid_client.setCallback([](const uavcan::ServiceCallResult<cvra::motor::config::PositionPID>& call_result)
    {
        if (call_result.isSuccessful() == false) {
            // TODO: error handling.
        }
    });

    uavcan::ServiceClient<cvra::motor::config::CurrentPID> current_pid_client(node);
    if (current_pid_client.init() < 0) {
        node_fail("cvra::motor::config::CurrentPID client");
    }

    current_pid_client.setCallback([](const uavcan::ServiceCallResult<cvra::motor::config::CurrentPID>& call_result)
    {
        if (call_result.isSuccessful() == false) {
            // TODO: error handling.
        }
    });

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(100));
        if (res < 0) {
            // log warning
        }

        if (palReadPad(GPIOA, GPIOA_BUTTON_WKUP)) {
            cvra::Reboot reboot_msg;
            reboot_msg.bootmode = reboot_msg.BOOTLOADER_TIMEOUT;
            reboot_pub.broadcast(reboot_msg);
        }

        for (int i = 0; i < SLAVE_CONFIG_COUNT; ++i) {
            if (parameter_namespace_contains_changed(&slave_configs[i].root)) {
                if (parameter_namespace_contains_changed(&slave_configs[i].speed_pid.root)) {
                    cvra::motor::config::SpeedPID::Request request;
                    request.pid.kp = parameter_scalar_get(&slave_configs[i].speed_pid.kp);
                    request.pid.ki = parameter_scalar_get(&slave_configs[i].speed_pid.ki);
                    request.pid.kd = parameter_scalar_get(&slave_configs[i].speed_pid.kd);
                    request.pid.ilimit = parameter_scalar_get(&slave_configs[i].speed_pid.ilimit);
                    speed_pid_client.call(i, request);
                }

                if (parameter_namespace_contains_changed(&slave_configs[i].position_pid.root)) {
                    cvra::motor::config::PositionPID::Request request;
                    request.pid.kp = parameter_scalar_get(&slave_configs[i].position_pid.kp);
                    request.pid.ki = parameter_scalar_get(&slave_configs[i].position_pid.ki);
                    request.pid.kd = parameter_scalar_get(&slave_configs[i].position_pid.kd);
                    request.pid.ilimit = parameter_scalar_get(&slave_configs[i].position_pid.ilimit);
                    position_pid_client.call(i, request);
                }

                if (parameter_namespace_contains_changed(&slave_configs[i].current_pid.root)) {
                    cvra::motor::config::CurrentPID::Request request;
                    request.pid.kp = parameter_scalar_get(&slave_configs[i].current_pid.kp);
                    request.pid.ki = parameter_scalar_get(&slave_configs[i].current_pid.ki);
                    request.pid.kd = parameter_scalar_get(&slave_configs[i].current_pid.kd);
                    request.pid.ilimit = parameter_scalar_get(&slave_configs[i].current_pid.ilimit);
                    current_pid_client.call(i, request);
                }
            }
        }

        cvra::motor::control::Velocity vel_ctrl_setpt;
        vel_ctrl_setpt.velocity = m1_vel_setpt;
        velocity_ctrl_setpt_pub.unicast(vel_ctrl_setpt, 10);
        vel_ctrl_setpt.velocity = m2_vel_setpt;
        velocity_ctrl_setpt_pub.unicast(vel_ctrl_setpt, 11);

        can_bridge_send_frames(node);

        // todo: publish time once a second
        // time_sync_master.publish();
    }
    return msg_t();
}

} // namespace uavcan_node

extern "C" {

void uavcan_node_start(uint8_t id)
{
    static uint8_t node_id = id;
    chThdCreateStatic(uavcan_node::thread_wa, UAVCAN_NODE_STACK_SIZE, NORMALPRIO, uavcan_node::main, &node_id);
}

} // extern "C"
