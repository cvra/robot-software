#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
#include <string>

void uavcan_node_start(std::string can_iface, uint8_t id);

extern "C" {
#endif

#include <stdint.h>
#include "bus_enumerator.h"

extern bus_enumerator_t bus_enumerator;

#ifdef __cplusplus
}
#endif

#endif /* UAVCAN_NODE_H */
