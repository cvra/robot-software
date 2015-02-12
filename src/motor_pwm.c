#include <ch.h>
#include <hal.h>
#include <stdlib.h>


#define PWM_PERIOD                  2880
#define PWM_DIRECTION_CHANNEL       0
#define PWM_POWER_CHANNEL           1

#define DIRECTION_DC_LOW            0
#define DIRECTION_DC_HIGH           PWM_PERIOD
#define DIRECTION_DC_RECHARGE       (0.75 * PWM_PERIOD)     // 10us
#define POWR_DC_RECHARGE_CORRECTION (PWM_PERIOD - DIRECTION_DC_RECHARGE)
#define RECHARGE_COUNTDOWN_RELOAD   50                      // every 2ms



/*
 * To prevent common-mode steps on the current measurement shunt resistor,
 * one side of the H-bridge is static (high or low), and the other does PWM.
 * The MOS-driver doesn't allow static high, so we have to switch to low every
 * 2ms for a duration of 10us to recharge the charge pump.
 *
 * Both H-bridge sides are connected to the same timer for PWM. The static side
 * is either at 0% or 100% duty cycle. In the case of 100%, every 2ms the duty
 * cycle is changed to 75% (resulting in a low time of 10us for a PWM frequency
 * of 25kHz) for a single cycle, to recharge the charge pump.
 *
 * At every 50th counter reset interrupt, the duty cycle of the static side is
 * changed from 100% to 75% (or back, depending on the recharge flag).
 */


static uint32_t power_pwm;

void pwm_counter_reset(PWMDriver *pwmd)
{
    static uint8_t recharge_countdown = RECHARGE_COUNTDOWN_RELOAD;
    uint32_t direction_dc = pwmd->tim->CCR[PWM_DIRECTION_CHANNEL];

    if (direction_dc == DIRECTION_DC_LOW) {
        recharge_countdown = RECHARGE_COUNTDOWN_RELOAD;
    } else if (direction_dc == DIRECTION_DC_RECHARGE) {
        pwmd->tim->CCR[PWM_DIRECTION_CHANNEL] = DIRECTION_DC_HIGH;
    }

    if (recharge_countdown == 0) {
#ifdef WHITENOISE_RECHARGE
        recharge_countdown = RECHARGE_COUNTDOWN_RELOAD - rand() % (RECHARGE_COUNTDOWN_RELOAD/2);
#else
        recharge_countdown = RECHARGE_COUNTDOWN_RELOAD;
#endif
        pwmd->tim->CCR[PWM_DIRECTION_CHANNEL] = DIRECTION_DC_RECHARGE;
        if (power_pwm < POWR_DC_RECHARGE_CORRECTION) {
            pwmd->tim->CCR[PWM_POWER_CHANNEL] = 0;
        } else {
            pwmd->tim->CCR[PWM_POWER_CHANNEL] = power_pwm - POWR_DC_RECHARGE_CORRECTION;
        }
    } else {
        pwmd->tim->CCR[PWM_POWER_CHANNEL] = power_pwm;
    }

    recharge_countdown--;       // don't forget this
}

static const PWMConfig pwm_cfg = {
    72000000,
    PWM_PERIOD,           // 25kHz
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

    if (dc < 0) {
        pwmEnableChannel(&PWMD1, PWM_DIRECTION_CHANNEL, DIRECTION_DC_HIGH);
        power_pwm = (1 + dc) * PWM_PERIOD;
    } else {
        pwmEnableChannel(&PWMD1, PWM_DIRECTION_CHANNEL, DIRECTION_DC_LOW);
        power_pwm = dc * PWM_PERIOD;
    }
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
