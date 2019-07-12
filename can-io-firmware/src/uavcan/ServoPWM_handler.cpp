#include <error/error.h>
#include "ServoPWM_handler.hpp"
#include "servo.h"
#include "main.h"

void ServoPWM_handler(const uavcan::ReceivedDataStructure<cvra::io::ServoPWM>& msg)
{
    float pos[4];
    float vel[4];
    float acc[4];

    if (msg.node_id != config.ID) {
        DEBUG("PWM message not addressed to us, dropping it.");
        return;
    }

    for (int i = 0; i < 4; i++) {
        pos[i] = msg.servo_pos[i];
        vel[i] = msg.servo_vel[i];
        acc[i] = msg.servo_acc[i];
    }

    NOTICE("Changing the PWM values: %.2f %.2f %.2f %.2f", pos[0], pos[1], pos[2], pos[3]);
    servo_set(pos, vel, acc);
}
