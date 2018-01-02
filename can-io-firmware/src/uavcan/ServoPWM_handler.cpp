#include "error/error.h"
#include "ServoPWM_handler.hpp"
#include "servo.h"
#include "main.h"

void ServoPWM_handler(const uavcan::ReceivedDataStructure<cvra::io::ServoPWM> &msg)
{
    float pos[4];

    if (msg.node_id != config.ID) {
        DEBUG("PWM message not addressed to us, dropping it.");
        return;
    }

    for (int i = 0; i < 4; i++) {
        pos[i] = msg.servo_pos[i];
    }

    NOTICE("Changing the PWM values: %.2f %.2f %.2f %.2f", pos[0], pos[1], pos[2], pos[3]);
    servo_set(pos);
}
