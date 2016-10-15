#ifndef UAVCAN_NODE_PRIVATE_HPP
#define UAVCAN_NODE_PRIVATE_HPP

typedef uavcan::Node<16384> Node;
namespace uavcan_node {
    Node& getNode();
}

#endif
