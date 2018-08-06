#include <ch.h>
#include <hal.h>
#include "motor_pwm.h" // to trigger charge pump recharge cycle
#include "metal_detector.h"
#include "analog.h"

#define ADC_MAX         4096
#define ADC_TO_AMPS     0.001611328125f // 3.3/4096/(0.01*50)
#define ADC_TO_VOLTS    0.005281575521f // 3.3/4096/(18/(100+18))

#define ADC_NB_CHANNELS 4
#define DMA_BUFFER_SIZE (380*2)         // dual buffer of 90 (see adc timing below)

event_source_t analog_event;

static int32_t battery_voltage;
static int32_t aux_in;


float analog_get_battery_voltage(void)
{
    return (float)battery_voltage * ADC_TO_VOLTS;
}

float analog_get_motor_current(void)
{
    return 0.0;
}

float analog_get_auxiliary(void)
{
    return (float)aux_in / (ADC_MAX * 2);
}

static void adc_callback(ADCDriver *adcp, adcsample_t *adc_samples, size_t n)
{
    (void)adcp;

    STM32_TIM15->CR1 &= ~STM32_TIM_CR1_CEN;    // turn ADC timer off

    battery_voltage = adc_samples[3];
    aux_in = adc_samples[0] + adc_samples[2];

    chSysLockFromISR();
    metal_detector_set_adc_samples(adc_samples, n);
    chEvtBroadcastFlagsI(&analog_event, ANALOG_EVENT_CONVERSION_DONE);
    chSysUnlockFromISR();
}

static THD_FUNCTION(adc_task, arg)
{
    (void)arg;
    chRegSetThreadName("adc read");
    static adcsample_t adc_samples[ADC_NB_CHANNELS * DMA_BUFFER_SIZE];
    static const ADCConversionGroup adcgrpcfg1 = {
        TRUE,                   // circular
        ADC_NB_CHANNELS,        // nb channels
        adc_callback,           // callback fn
        NULL,                   // error callback fn
        ADC_CFGR_EXTSEL_SRC(14) | ADC_CFGR_EXTEN_RISING, // CFGR : External trigger from TIM15_TRGO event
        0,                      // TR1
        6,                      // CCR : DUAL=regular,simultaneous
        /* ADC timing
         * 601.5 sampling + 12.5 conversion ADC cycles (72MHz)
         * sampling time -> ~8.5us (max frequency ~117kHz)
         * with 2 samples per conversion -> ~58kHz max frequency
         */
        {ADC_SMPR1_SMP_AN1(7), 0}, // SMPRx : 601.5 sampling cycles
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {ADC_SMPR1_SMP_AN2(7) | ADC_SMPR1_SMP_AN3(7), 0},   // SSMPRx : 601.5 sampling cycles
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
    };

    adcStart(&ADCD1, NULL);

    // fix for ChibiOS ADC config bug
    ADCD1.adcc->CCR = STM32_ADC_ADC12_CLOCK_MODE | ADC_CCR_MDMA_WORD |
                      (ADCD1.adcc->CCR & ~(ADC_CCR_CKMODE_MASK | ADC_CCR_MDMA_MASK));

    // Setup TIM8 to generate conversion trigger at desired frequency
    rccEnableTIM15(FALSE);  // enable timer 15; not in low power mode
    rccResetTIM15();
    STM32_TIM15->CR2    = STM32_TIM_CR2_MMS(2);     // TRGO on update event
    STM32_TIM15->PSC    = 0;                        // No Prescaler.
    STM32_TIM15->SR     = 0;                        // Clear pending IRQs.
    STM32_TIM15->DIER   = 0;                        // DMA-related DIER bits.
    STM32_TIM15->SMCR   = 0;
    STM32_TIM15->CCMR1  = 0;
    STM32_TIM15->CCER   = 0;
    STM32_TIM15->ARR    = 15000;                    // Preload to count to 72MHz/4.8kHz
    STM32_TIM15->CR1    = 0;                        // Don't start yet

    adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, DMA_BUFFER_SIZE); // should never return
}

void analog_init(void)
{
    static THD_WORKING_AREA(adc_task_wa, 256);
    chEvtObjectInit(&analog_event);
    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), HIGHPRIO, adc_task, NULL);
}
