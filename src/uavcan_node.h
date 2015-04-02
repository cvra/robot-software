#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void uavcan_node_start(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* UAVCAN_NODE_H */