#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <blocking_uart_driver.h>
#include <stdlib.h>

BlockingUARTDriver blocking_uart_stream;
BaseSequentialStream* stderr = (BaseSequentialStream*)&blocking_uart_stream;


#define PWM_PERIOD                  2880
#define PWM_DIRECTION_CHANNEL       0
#define PWM_POWER_CHANNEL           1

#define DIRECTION_DC_LOW            0
#define DIRECTION_DC_HIGH           PWM_PERIOD
#define DIRECTION_DC_RECHARGE       (0.75 * PWM_PERIOD)     // 10us
#define POWR_DC_RECHARGE_CORRECTION (PWM_PERIOD - DIRECTION_DC_RECHARGE)
#define RECHARGE_COUNTDOWN_RELOAD   50                      // every 2ms

#define ADC_MAX         4096
#define ADC_TO_AMPS     0.001611328125f
#define ADC_TO_VOLTS    0.005283043033f


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
        recharge_countdown = RECHARGE_COUNTDOWN_RELOAD - rand() % (RECHARGE_COUNTDOWN_RELOAD/2);
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

static void pwm_setup(void)
{
    pwmStart(&PWMD1, &pwm_cfg);
    pwmEnablePeriodicNotification(&PWMD1);
}

static void set_pwm(float dc)
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


static THD_WORKING_AREA(quad_task_wa, 128);
static THD_FUNCTION(quad_task, arg)
{
    (void)arg;
    chRegSetThreadName("encoder read");
    rccEnableTIM4(FALSE);           // enable timer 4
    rccResetTIM4();
    STM32_TIM4->CR2    = 0;
    STM32_TIM4->PSC    = 0;                         // Prescaler value.
    STM32_TIM4->SR     = 0;                         // Clear pending IRQs.
    STM32_TIM4->DIER   = 0;                         // DMA-related DIER bits.
    STM32_TIM4->SMCR   = STM32_TIM_SMCR_SMS(3);     // count on both edges
    STM32_TIM4->CCMR1  = STM32_TIM_CCMR1_CC1S(1);   // CC1 channel is input, IC1 is mapped on TI1
    STM32_TIM4->CCMR1 |= STM32_TIM_CCMR1_CC2S(1);   // CC2 channel is input, IC2 is mapped on TI2
    STM32_TIM4->CCER   = 0;
    STM32_TIM4->ARR    = 0xFFFF;
    STM32_TIM4->CR1    = 1;                         // start

    while(42){
        chThdSleepMilliseconds(100);
    }
    return 0;
}

static THD_WORKING_AREA(ext_quad_task_wa, 128);
static THD_FUNCTION(ext_quad_task, arg)
{
    (void)arg;
    chRegSetThreadName("external encoder read");
    palSetPadMode(GPIOB, GPIOB_GPIO_A, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOB, GPIOB_GPIO_B, PAL_MODE_ALTERNATE(2));

    rccEnableTIM3(FALSE);           // enable timer 3
    rccResetTIM3();
    STM32_TIM3->CR2    = 0;
    STM32_TIM3->PSC    = 0;                         // Prescaler value.
    STM32_TIM3->SR     = 0;                         // Clear pending IRQs.
    STM32_TIM3->DIER   = 0;                         // DMA-related DIER bits.
    STM32_TIM3->SMCR   = STM32_TIM_SMCR_SMS(3);     // count on both edges
    STM32_TIM3->CCMR1  = STM32_TIM_CCMR1_CC1S(1);   // CC1 channel is input, IC1 is mapped on TI1
    STM32_TIM3->CCMR1 |= STM32_TIM_CCMR1_CC2S(1);   // CC2 channel is input, IC2 is mapped on TI2
    STM32_TIM3->CCER   = 0;
    STM32_TIM3->ARR    = 0xFFFF;
    STM32_TIM3->CR1    = 1;                         // start

    while(42){
        chThdSleepMilliseconds(100);
    }
    return 0;
}


static THD_WORKING_AREA(adc_task_wa, 256);
static THD_FUNCTION(adc_task, arg)
{
    (void)arg;
    chRegSetThreadName("adc read");
    static adcsample_t adc_samples[4];
    static const ADCConversionGroup adcgrpcfg1 = {
        FALSE,                  // circular?
        4,                      // nb channels
        NULL,                   // callback fn
        NULL,                   // error callback fn
        0,                      // CFGR
        0,                      // TR1
        6,                      // CCR : DUAL=regualar,simultaneous
        {ADC_SMPR1_SMP_AN1(0), 0},                          // SMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {ADC_SMPR1_SMP_AN2(0) | ADC_SMPR1_SMP_AN3(0), 0},   // SSMPRx : sample time minimum
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
    };

    while (1) {
        int i;
        float mean_current = 0.0f;
        for(i = 0; i < 1000; i++){
            adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, 1);
            mean_current += (adc_samples[1]-ADC_MAX/2)*ADC_TO_AMPS;
        }
        mean_current /= 1000;

        chThdSleepMilliseconds(100);
    }
    return 0;
}

void panic_hook(const char* reason)
{
    palClearPad(GPIOA, GPIOA_LED);      // turn on LED (active low)
    int i;
    while(42){
        for(i = 10000000; i>0; i--){
            __asm__ volatile ("nop");
        }
        chprintf(stderr, "%s\n", reason);
    }
}


int main(void) {
    halInit();
    chSysInit();

    pwm_setup();

    set_pwm(0.0);

    palSetPad(GPIOA, GPIOA_MOTOR_EN_A);
    palSetPad(GPIOA, GPIOA_MOTOR_EN_B);

    adcStart(&ADCD1, NULL);

    blocking_uart_init(&blocking_uart_stream, USART3, 115200);

    chprintf(stderr, "boot\n");

    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), LOWPRIO, adc_task, NULL);
    chThdCreateStatic(quad_task_wa, sizeof(quad_task_wa), LOWPRIO, quad_task, NULL);
    chThdCreateStatic(ext_quad_task_wa, sizeof(ext_quad_task_wa), LOWPRIO, ext_quad_task, NULL);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        set_pwm(-0.2);
        chThdSleepMilliseconds(1000);
        palClearPad(GPIOA, GPIOA_LED);
        set_pwm(0.2);
        chThdSleepMilliseconds(1000);
    }
}
