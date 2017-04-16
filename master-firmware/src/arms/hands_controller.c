#include "main.h"

#include "can/motor_manager.h"
#include "can/hand_driver.h"

#include "base/base_controller.h"
#include "arms/arms_controller.h"
#include "cvra_arm_motors.h"
#include "hands_controller.h"

hand_t left_hand;


void hands_init()
{
    hand_init(&left_hand);
    static cvra_arm_motor_t left_wrist = {.m = &motor_manager, .direction = 1, .index = 0};

    hand_set_wrist_callbacks(&left_hand, set_left_wrist_position, get_left_wrist_position, &left_wrist);

    hand_set_related_robot_pos(&left_hand, &robot.pos);
    hand_set_related_arm(&left_hand, &left_arm);
}
