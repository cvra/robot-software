#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

void uavcan_start(unsigned int node_id, const char *node_name);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>
typedef uavcan::Node<4096> Node;
#endif

#endif
