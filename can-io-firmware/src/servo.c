#include <string.h>

#include "pwm.h"
#include "servo.h"
#include <quadramp/quadramp.h>

#define SERVO_PWM_THREAD_FREQ 1000 // 1kHz
#define SERVO_PWM_TIMER_FREQ 1000000 // 1MHz
#define SERVO_PWM_PERIOD 20000 // 20 ms period
#define SERVO_PWM_RAMP_SCALE (SERVO_PWM_TIMER_FREQ / SERVO_PWM_THREAD_FREQ)

typedef struct {
    float setpoint; // Duty cycle setpoint
    struct quadramp_filter filter;
} servo_t;

static servo_t servo[4];
static mutex_t servo_mutex;

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

/* Negative coefficients are "do not change" */
static void set_quadramp_coefficients(struct quadramp_filter* filter, float vel, float acc)
{
    if (vel > 0) {
        quadramp_set_1st_order_vars(filter, vel * SERVO_PWM_RAMP_SCALE, vel * SERVO_PWM_RAMP_SCALE);
    }
    if (acc > 0) {
        quadramp_set_2nd_order_vars(filter, acc * SERVO_PWM_RAMP_SCALE, acc * SERVO_PWM_RAMP_SCALE);
    }
}

void servo_set(const float pos[4], const float vel[4], const float acc[4])
{
    chMtxLock(&servo_mutex);

    servo[0].setpoint = pos[0];
    servo[1].setpoint = pos[1];
    servo[2].setpoint = pos[2];
    servo[3].setpoint = pos[3];

    set_quadramp_coefficients(&servo[0].filter, vel[0], acc[0]);
    set_quadramp_coefficients(&servo[1].filter, vel[1], acc[1]);
    set_quadramp_coefficients(&servo[2].filter, vel[2], acc[2]);
    set_quadramp_coefficients(&servo[3].filter, vel[3], acc[3]);

    chMtxUnlock(&servo_mutex);
}

void servo_init(void)
{
    pwm_init(SERVO_PWM_TIMER_FREQ, SERVO_PWM_PERIOD);

    memset(&servo, 0, sizeof(servo));
    chMtxObjectInit(&servo_mutex);

    quadramp_init(&servo[0].filter);
    quadramp_init(&servo[1].filter);
    quadramp_init(&servo[2].filter);
    quadramp_init(&servo[3].filter);

    set_quadramp_coefficients(&servo[0].filter, 1, 1);
    set_quadramp_coefficients(&servo[1].filter, 1, 1);
    set_quadramp_coefficients(&servo[2].filter, 1, 1);
    set_quadramp_coefficients(&servo[3].filter, 1, 1);
}

THD_FUNCTION(servo_thd, arg)
{
    (void)arg;
    uint32_t pulsewidth[4];

    while (1) {
        chMtxLock(&servo_mutex);

        pulsewidth[0] = quadramp_do_filter(&servo[0].filter, duty_cycle(servo[0].setpoint));
        pulsewidth[1] = quadramp_do_filter(&servo[1].filter, duty_cycle(servo[1].setpoint));
        pulsewidth[2] = quadramp_do_filter(&servo[2].filter, duty_cycle(servo[2].setpoint));
        pulsewidth[3] = quadramp_do_filter(&servo[3].filter, duty_cycle(servo[3].setpoint));

        pwm_set_pulsewidth(PWM_CHANNEL_0, pulsewidth[0]);
        pwm_set_pulsewidth(PWM_CHANNEL_1, pulsewidth[1]);
        pwm_set_pulsewidth(PWM_CHANNEL_2, pulsewidth[2]);
        pwm_set_pulsewidth(PWM_CHANNEL_3, pulsewidth[3]);

        chMtxUnlock(&servo_mutex);

        chThdSleepMilliseconds(1000 / SERVO_PWM_THREAD_FREQ);
    }
}

void servo_start(void)
{
    static THD_WORKING_AREA(servo_thd_wa, 256);
    chThdCreateStatic(servo_thd_wa, sizeof(servo_thd_wa), NORMALPRIO, servo_thd, NULL);
}
