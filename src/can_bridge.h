#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>
#include <serial-can-bridge/serial_can_bridge.h>

// block allocator memory pools for CAN frames
extern memory_pool_t can_bridge_rx_pool;
extern memory_pool_t can_bridge_tx_pool;
// message queues for CAN frame pointers
extern mailbox_t can_bridge_rx_queue;
extern mailbox_t can_bridge_tx_queue;

extern semaphore_t can_bridge_is_initialized;

void can_bridge_init(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
