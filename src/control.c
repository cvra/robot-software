#include "motor_pwm.h"
#include "control.h"

#define LOW_BATT_TH 9.0f // [V]

static void motor_set_voltage(float u)
{
    float u_batt = 0; //TODO
    if (u_batt > LOW_BATT_TH) {
        motor_pwm_enable();
        motor_pwm_set(u / u_batt);
    } else {
        motor_pwm_disable();
    }
}

