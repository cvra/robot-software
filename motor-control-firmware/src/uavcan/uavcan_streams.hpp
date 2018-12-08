#ifndef UAVCAN_STREAMS_HPP
#define UAVCAN_STREAMS_HPP

#include "uavcan_node.h"

/** Starts the various state publishers. */
int uavcan_streams_start(Node& node);

/** Must be called in the UAVCAN loop. */
void uavcan_streams_spin(Node& node);

#endif
