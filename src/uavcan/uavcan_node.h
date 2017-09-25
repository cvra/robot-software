#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define UAVCAN_SPIN_FREQUENCY   100

void uavcan_node_start(uint8_t id, const char *name);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>

/** Returns the UAVCAN node. */
typedef uavcan::Node<4096> Node;

#endif

#endif /* UAVCAN_NODE_H */
