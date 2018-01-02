#include <string.h>

#include "pwm.h"
#include "servo.h"
#include "quadramp/quadramp.h"

#define SERVO_PWM_TIMER_FREQ    1000000 // 1MHz
#define SERVO_PWM_PERIOD        20000   // 20 ms period

typedef struct {
    float setpoint; // Duty cycle setpoint
    struct quadramp_filter filter;
} servo_t;

static servo_t servo[4];

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

void servo_set(const float pos[4], const float vel[4], const float acc[4])
{
    servo[0].setpoint = pos[0];
    servo[1].setpoint = pos[1];
    servo[2].setpoint = pos[2];
    servo[3].setpoint = pos[3];

    quadramp_set_1st_order_vars(&servo[0].filter, vel[0] * SERVO_PWM_TIMER_FREQ, vel[0] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_1st_order_vars(&servo[1].filter, vel[1] * SERVO_PWM_TIMER_FREQ, vel[1] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_1st_order_vars(&servo[2].filter, vel[2] * SERVO_PWM_TIMER_FREQ, vel[2] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_1st_order_vars(&servo[3].filter, vel[3] * SERVO_PWM_TIMER_FREQ, vel[3] * SERVO_PWM_TIMER_FREQ);

    quadramp_set_2nd_order_vars(&servo[0].filter, acc[0] * SERVO_PWM_TIMER_FREQ, acc[0] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_2nd_order_vars(&servo[1].filter, acc[1] * SERVO_PWM_TIMER_FREQ, acc[1] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_2nd_order_vars(&servo[2].filter, acc[2] * SERVO_PWM_TIMER_FREQ, acc[2] * SERVO_PWM_TIMER_FREQ);
    quadramp_set_2nd_order_vars(&servo[3].filter, acc[3] * SERVO_PWM_TIMER_FREQ, acc[3] * SERVO_PWM_TIMER_FREQ);
}

void servo_init(void)
{
    pwm_init(SERVO_PWM_TIMER_FREQ, SERVO_PWM_PERIOD);

    memset(&servo, 0, sizeof(servo));

    quadramp_init(&servo[0].filter);
    quadramp_init(&servo[1].filter);
    quadramp_init(&servo[2].filter);
    quadramp_init(&servo[3].filter);
}

THD_FUNCTION(servo_thd, arg)
{
    (void) arg;
    uint32_t pulsewidth[4];

    while (1) {
        pulsewidth[0] = quadramp_do_filter(&servo[0].filter, duty_cycle(servo[0].setpoint));
        pulsewidth[1] = quadramp_do_filter(&servo[1].filter, duty_cycle(servo[1].setpoint));
        pulsewidth[2] = quadramp_do_filter(&servo[2].filter, duty_cycle(servo[2].setpoint));
        pulsewidth[3] = quadramp_do_filter(&servo[3].filter, duty_cycle(servo[3].setpoint));

        pwm_set_pulsewidth(PWM_CHANNEL_0, pulsewidth[0]);
        pwm_set_pulsewidth(PWM_CHANNEL_1, pulsewidth[1]);
        pwm_set_pulsewidth(PWM_CHANNEL_2, pulsewidth[2]);
        pwm_set_pulsewidth(PWM_CHANNEL_3, pulsewidth[3]);

        chThdSleepMilliseconds(1);
    }
}

void servo_start(void)
{
    static THD_WORKING_AREA(servo_thd_wa, 256);
    chThdCreateStatic(servo_thd_wa, sizeof(servo_thd_wa), NORMALPRIO, servo_thd, NULL);
}
