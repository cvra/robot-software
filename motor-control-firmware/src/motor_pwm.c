#include <ch.h>
#include <hal.h>
#include <stdlib.h>
#include <math.h>


#define PWM_PERIOD                  28800
#define PWM_DIRECTION_CHANNEL       0
#define PWM_POWER_CHANNEL           1

#define DIRECTION_DC_LOW            0
#define DIRECTION_DC_HIGH           PWM_PERIOD


static int32_t power_pwm;

void pwm_counter_reset(PWMDriver *pwmd)
{
    if (power_pwm >= 0) { // forward direction (no magic)
        pwmd->tim->CCR[PWM_DIRECTION_CHANNEL] = DIRECTION_DC_LOW;
        pwmd->tim->CCR[PWM_POWER_CHANNEL] = power_pwm;
        STM32_TIM15->CR1 |= STM32_TIM_CR1_CEN;      // enable ADC timing timer
    }
}

static const PWMConfig pwm_cfg = {
    360000,             // clock frequency in Hz (sets prescaler)
    PWM_PERIOD,         // 12.5Hz
    pwm_counter_reset,
    // activate channel 1 and 2
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,                  // TIMx_CR2 value
    0                   // TIMx_DIER value
};

void motor_pwm_setup(void)
{
    pwmStart(&PWMD1, &pwm_cfg);
    pwmEnablePeriodicNotification(&PWMD1);
}

void motor_pwm_set(float dc)
{
    if (dc > 0.95) {
        dc = 0.95;
    } else if (dc < -0.95){
        dc = -0.95;
    }

    power_pwm = dc * PWM_PERIOD;
}

void motor_pwm_enable(void)
{
    palSetPad(GPIOA, GPIOA_MOTOR_EN_A);
    palSetPad(GPIOA, GPIOA_MOTOR_EN_B);
}

void motor_pwm_disable(void)
{
    motor_pwm_set(0);
    palClearPad(GPIOA, GPIOA_MOTOR_EN_A);
    palClearPad(GPIOA, GPIOA_MOTOR_EN_B);
}
