#include "error/error.h"
#include "ServoPWM_handler.hpp"
#include "servo_pwm.h"
#include "bootloader_config.h"

void ServoPWM_handler(const uavcan::ReceivedDataStructure<cvra::io::ServoPWM> &msg)
{
    bootloader_config_t cfg;
    float pos[4];

    if (!config_get(&cfg)) {
        ERROR("Cannot load config!");
        return;
    }

    if (msg.node_id != cfg.ID) {
        DEBUG("PWM message not addressed to us, dropping it.");
        return;
    }

    for (int i = 0; i < 4; i++) {
        pos[i] = msg.servo_pos[i];
    }

    NOTICE("Changing the PWM values: %.2f %.2f %.2f %.2f", pos[0], pos[1], pos[2], pos[3]);
    servo_set(pos);
}
