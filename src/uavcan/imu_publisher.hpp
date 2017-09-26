#ifndef IMU_PUBLISHER_HPP
#define IMU_PUBLISHER_HPP

#include "uavcan/uavcan_node.h"

void imu_publisher_start(Node &node);
void imu_publisher_spin(Node &node);

#endif
