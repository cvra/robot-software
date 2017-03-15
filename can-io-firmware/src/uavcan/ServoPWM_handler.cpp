#include "error/error.h"
#include "ServoPWM_handler.hpp"

void ServoPWM_handler(const uavcan::ReceivedDataStructure<cvra::io::ServoPWM> &msg)
{
    NOTICE("Changing the PWM value");
}
