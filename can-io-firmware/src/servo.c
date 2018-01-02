#include <string.h>

#include "pwm.h"
#include "servo.h"

#define SERVO_PWM_TIMER_FREQ    1000000 // 1MHz
#define SERVO_PWM_PERIOD        20000   // 20 ms period

struct servos_t {
    float setpoint[4]; // Duty cycle setpoints
};

static struct servos_t servos;

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
    servos.setpoint[0] = pos[0];
    servos.setpoint[1] = pos[1];
    servos.setpoint[2] = pos[2];
    servos.setpoint[3] = pos[3];
}

void servo_init(void)
{
    memset(&servos, 0, sizeof(struct servos_t));
    pwm_init(SERVO_PWM_TIMER_FREQ, SERVO_PWM_PERIOD);
}

THD_FUNCTION(servo_thd, arg)
{
    (void) arg;
    while (1) {
        pwm_set_pulsewidth(PWM_CHANNEL_0, duty_cycle(servos.setpoint[0]));
        pwm_set_pulsewidth(PWM_CHANNEL_1, duty_cycle(servos.setpoint[1]));
        pwm_set_pulsewidth(PWM_CHANNEL_2, duty_cycle(servos.setpoint[2]));
        pwm_set_pulsewidth(PWM_CHANNEL_3, duty_cycle(servos.setpoint[3]));
    }
}

void servo_start(void)
{
    static THD_WORKING_AREA(servo_thd_wa, 256);
    chThdCreateStatic(servo_thd_wa, sizeof(servo_thd_wa), NORMALPRIO, servo_thd, NULL);
}
