/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <ch.hpp>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <src/can_bridge.h>

#include <errno.h>

#include <uavcan/protocol/global_time_sync_master.hpp>

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
    if (!can_bridge_is_initialized) {
        return;
    }
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

            uavcan::MonotonicTime timeout = node.getMonotonicTime();
            timeout += uavcan::MonotonicDuration::fromMSec(100);

            uavcan::CanSelectMasks masks;
            masks.write = 1;
            can.driver.select(masks, timeout);
            if (masks.write & 1) {
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

    node.setStatusOk();

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(10));
        if (res < 0) {
            // log warning
        }

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
