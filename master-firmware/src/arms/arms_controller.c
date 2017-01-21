#include "priorities.h"
#include "arms_controller.h"
#include "cvra_arm_motors.h"

#define ARMS_CONTROLLER_STACKSIZE 1024

scara_t left_arm;
scara_t right_arm;

void arms_init(void)
{
    scara_init(&left_arm);
    scara_set_shoulder_callback(&left_arm, set_left_shoulder_position);
    scara_set_elbow_callback(&left_arm, set_left_elbow_position);
    scara_set_physical_parameters(&left_arm, 140.f, 72.f); // Arm lengths in mm

    scara_init(&right_arm);
    scara_set_shoulder_callback(&right_arm, set_right_shoulder_position);
    scara_set_elbow_callback(&right_arm, set_right_elbow_position);
    scara_set_physical_parameters(&right_arm, 140.f, 72.f); // Arm lengths in mm
}


static THD_FUNCTION(arms_ctrl_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while (1) {
        /* Wait 10 milliseconds (100 Hz) */
        chThdSleepMilliseconds(1000 / ARMS_FREQUENCY);
    }
}

void arms_controller_start(void)
{
    static THD_WORKING_AREA(arms_ctrl_thd_wa, ARMS_CONTROLLER_STACKSIZE);
    chThdCreateStatic(arms_ctrl_thd_wa, sizeof(arms_ctrl_thd_wa), ARMS_CONTROLLER_PRIO, arms_ctrl_thd, NULL);
}
