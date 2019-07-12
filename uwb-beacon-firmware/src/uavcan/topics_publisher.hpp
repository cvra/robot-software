#ifndef TOPICS_PUBLISHER_HPP
#define TOPICS_PUBLISHER_HPP

#include "./uavcan/uavcan_node.h"

/** This module streams topics from the internal messagebus to the UAVCAN bus. */
void topics_publisher_start(Node& node);
void topics_publisher_spin(Node& node);

#endif
