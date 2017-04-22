#ifndef ROCKET_DRIVER_H
#define ROCKET_DRIVER_H

/* TODO Calibrate this. */
#define ROCKET_POS_RELEASE  0.0015
#define ROCKET_POS_LOCK     0.0021

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>
int rocket_init(uavcan::INode &node);
#endif

#ifdef __cplusplus
extern "C" {
#endif

void rocket_set_pos(float pos);

/** Program a timer to launch the rocket after given time in seconds */
void rocket_program_launch_time(unsigned int time);

#ifdef __cplusplus
}
#endif
#endif
