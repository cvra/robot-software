#include <string.h>

#include "pwm.h"
#include "servo.h"
#include "quadramp/quadramp.h"

#define SERVO_PWM_THREAD_FREQ 100 /* hz */
#define SERVO_PWM_TIMER_FREQ 1000000 /* hz */
#define SERVO_PWM_PERIOD 20000 // 20 ms period
#define SERVO_PWM_RAMP_SCALE (SERVO_PWM_TIMER_FREQ / SERVO_PWM_THREAD_FREQ)

typedef struct {
    float setpoint;
    struct quadramp_filter filter;
} servo_state;

/* Mapping from servo channel to timer channel  */
static int pwm_channels[SERVO_COUNT] = {1, 2};

static servo_state servo[SERVO_COUNT];

static MUTEX_DECL(servo_mutex);

/* convert 0.0-1.0 to full pwm on period. */
static uint32_t duty_cycle(float pos)
{
    if (pos > 1) {
        pos = 1;
    } else if (pos < 0) {
        pos = 0;
    }
    return (uint32_t)(pos * SERVO_PWM_PERIOD);
}

static void set_quadramp_coefficients(struct quadramp_filter* filter, float vel, float acc)
{
    quadramp_set_1st_order_vars(filter, vel * SERVO_PWM_RAMP_SCALE, vel * SERVO_PWM_RAMP_SCALE);
    quadramp_set_2nd_order_vars(filter, acc * SERVO_PWM_RAMP_SCALE, acc * SERVO_PWM_RAMP_SCALE);
}

void servo_set(int index, float pos, float vel, float acc)
{
    chMtxLock(&servo_mutex);

    servo[index].setpoint = pos;
    set_quadramp_coefficients(&servo[index].filter, vel, acc);

    chMtxUnlock(&servo_mutex);
}

static void timer_init(void)
{
    /* Timer1 channel 2 and 3 are connected to the servomotors. */
    rccEnableAPB2(RCC_APB2ENR_TIM1EN, true);
    rccResetAPB2(RCC_APB2RSTR_TIM1RST);
    pwm_setup_channels(STM32_TIM1,
                       PWM_CHANNEL(2) | PWM_CHANNEL(3),
                       STM32_TIMCLK2,
                       SERVO_PWM_TIMER_FREQ,
                       SERVO_PWM_PERIOD);
}

static void servo_init(void)
{
    timer_init();

    memset(&servo, 0, sizeof(servo));

    for (int i = 0; i < SERVO_COUNT; i++) {
        quadramp_init(&servo[i].filter);
        set_quadramp_coefficients(&servo[i].filter, 1, 1);
    }
}

THD_FUNCTION(servo_thd, arg)
{
    (void)arg;
    uint32_t pulsewidth;

    chRegSetThreadName("servo");

    while (1) {
        chMtxLock(&servo_mutex);

        for (int i = 0; i < SERVO_COUNT; i++) {
            pulsewidth = quadramp_do_filter(&servo[i].filter, duty_cycle(servo[i].setpoint));
            pwm_set_pulsewidth(STM32_TIM1, pwm_channels[i], pulsewidth);
        }

        chMtxUnlock(&servo_mutex);

        chThdSleepMilliseconds(1000 / SERVO_PWM_THREAD_FREQ);
    }
}

void servo_start(void)
{
    static THD_WORKING_AREA(servo_thd_wa, 512);

    servo_init();

    chThdCreateStatic(servo_thd_wa, sizeof(servo_thd_wa), NORMALPRIO, servo_thd, NULL);
}
