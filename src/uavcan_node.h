#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bus_enumerator.h"

void uavcan_node_start(uint8_t id);

void uavcan_reboot_nodes(void);

extern bus_enumerator_t bus_enumerator;

#ifdef __cplusplus
}
#endif

#endif /* UAVCAN_NODE_H */
