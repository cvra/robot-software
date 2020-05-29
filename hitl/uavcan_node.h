#ifndef UAVCAN_NODE_H
#define UAVCAN_NODE_H

#include <uavcan/uavcan.hpp>

constexpr unsigned NodeMemoryPoolSize = 16384;

typedef uavcan::Node<NodeMemoryPoolSize> Node;

#endif
