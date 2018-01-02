#include "pwm.h"
#include "servo.h"

/* convert 0.0-1.0 to full pwm duty cycle. */
static uint32_t duty_cycle(float pos)
{
    if (pos > 1) {
        pos = 1;
    } else if (pos < 0) {
        pos = 0;
    }
    return (uint32_t)(pos * SERVO_PWM_TIMER_FREQ);
}

void servo_set(const float pos[4])
{
    pwm_set_duty_cycle(STM32_TIM16, 0, duty_cycle(pos[0]));
    pwm_set_duty_cycle(STM32_TIM17, 0, duty_cycle(pos[1]));
    pwm_set_duty_cycle(STM32_TIM1, 1, duty_cycle(pos[2]));
    pwm_set_duty_cycle(STM32_TIM1, 2, duty_cycle(pos[3]));
}

void servo_init(void)
{
    pwm_init();
}
