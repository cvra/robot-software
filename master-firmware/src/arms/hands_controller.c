#include <ch.h>
#include "priorities.h"
#include "main.h"

#include "can/motor_manager.h"
#include "can/hand_driver.h"

#include "base/base_controller.h"
#include "arms/arms_controller.h"
#include "cvra_arm_motors.h"
#include "hands_controller.h"


#define HANDS_CONTROLLER_STACKSIZE 128


hand_t left_hand;
hand_t right_hand;


void hands_init(void)
{
    hand_init(&left_hand);
    static cvra_arm_motor_t left_wrist = {.m = &motor_manager, .direction = 1, .index = 0};

    hand_set_wrist_callbacks(&left_hand, set_left_wrist_position, get_left_wrist_position, &left_wrist);
    hand_set_fingers_callbacks(&left_hand, hand_driver_set_left_fingers);

    hand_set_related_robot_pos(&left_hand, &robot.pos);
    hand_set_related_arm(&left_hand, &left_arm);

    hand_init(&right_hand);
    static cvra_arm_motor_t right_wrist = {.m = &motor_manager, .direction = 1, .index = 0};

    hand_set_wrist_callbacks(&right_hand, set_right_wrist_position, get_right_wrist_position, &right_wrist);
    hand_set_fingers_callbacks(&right_hand, hand_driver_set_right_fingers);

    hand_set_related_robot_pos(&right_hand, &robot.pos);
    hand_set_related_arm(&right_hand, &right_arm);
}

static THD_FUNCTION(hands_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while (true) {
        hand_manage(&left_hand);
        hand_manage(&right_hand);

        chThdSleepMilliseconds(1000 / HANDS_FREQUENCY);
    }
}

void hands_controller_start(void)
{
    static THD_WORKING_AREA(hands_ctrl_thd_wa, HANDS_CONTROLLER_STACKSIZE);
    chThdCreateStatic(hands_ctrl_thd_wa, sizeof(hands_ctrl_thd_wa), HANDS_CONTROLLER_PRIO, hands_ctrl_thd, NULL);
}
