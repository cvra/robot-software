#ifndef CAN_IO_H
#define CAN_IO_H

#ifdef __cplusplus
extern "C" {
#endif

void can_io_set_pwm(const char* can_io_name, int channel, float pwm);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>
int can_io_init(uavcan::INode& node);
#endif

#endif /* CAN_IO_H */
