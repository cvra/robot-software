#ifndef TRAJECTORY_HANDLER_HPP
#define TRAJECTORY_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Trajectory.hpp>

void Trajectory_handler(const uavcan::ReceivedDataStructure<cvra::motor::control::Trajectory> &msg);


#endif
