#ifndef ROCKET_DRIVER_H
#define ROCKET_DRIVER_H

/* TODO Calibrate this. */
#define ROCKET_POS_OPEN 0.
#define ROCKET_POS_CLOSE 0.

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>
int rocket_init(uavcan::INode &node);
#endif

#ifdef __cplusplus
extern "C" {
#endif

void rocket_set_pos(float pos);

#ifdef __cplusplus
}
#endif
#endif
