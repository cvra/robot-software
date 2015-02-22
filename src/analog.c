#include <ch.h>
#include <hal.h>
#include <filter/iir.h>

#define ADC_MAX         4096
#define ADC_TO_AMPS     0.001611328125f // 3.3/4096/(0.01*50)
#define ADC_TO_VOLTS    0.005281575521f // 3.3/4096/(18/(100+18))

#define ADC_NB_CHANNELS 4
#define DMA_BUFFER_SIZE (234*1)
#define CURRENT_PRE_FILTER_WINDOW_SIZE  1

static int32_t motor_current;
static float motor_current_f;
static float battery_voltage;
static float aux_in;


float analog_get_battery_voltage(void)
{
    return battery_voltage;
}

float analog_get_motor_current(void)
{
    motor_current_f = motor_current_f * 0.85 + (- ((float)motor_current * 2 / DMA_BUFFER_SIZE - ADC_MAX / 2) * ADC_TO_AMPS * 0.15);
    return motor_current_f;
}

float analog_get_auxiliary(void)
{
    return aux_in;
}

static void adc_callback(ADCDriver *adcp, adcsample_t *adc_samples, size_t n)
{
    (void)adcp;
    uint32_t accumulator = 0;

    int i;
    for (i = 0; i < (int)(n * ADC_NB_CHANNELS); i += ADC_NB_CHANNELS) {
        accumulator += adc_samples[i + 1];
    }

    motor_current = accumulator;

    battery_voltage = adc_samples[3] * ADC_TO_VOLTS;
    aux_in = (float)(adc_samples[0] + adc_samples[2])/(ADC_MAX*2);
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
        ADC_CFGR_CONT,          // CFGR
        0,                      // TR1
        6,                      // CCR : DUAL=regular,simultaneous
        {ADC_SMPR1_SMP_AN1(7), 0},                          // SMPRx : sample time maximum -> ~117kHz
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(1) | ADC_SQR1_SQ2_N(1), 0, 0, 0}, // SQRx : ADC1_CH1 2x
        {ADC_SMPR1_SMP_AN2(7) | ADC_SMPR1_SMP_AN3(7), 0},   // SSMPRx : sample time maximum -> ~117kHz
        {ADC_SQR1_NUM_CH(2) | ADC_SQR1_SQ1_N(2) | ADC_SQR1_SQ2_N(3), 0, 0, 0}, // SSQRx : ADC2_CH2, ADC2_CH3
    };

    adcStart(&ADCD1, NULL);

    motor_current_f = 0;

    adcConvert(&ADCD1, &adcgrpcfg1, adc_samples, DMA_BUFFER_SIZE); // should never return

    return 0;
}

void analog_init(void)
{
    static THD_WORKING_AREA(adc_task_wa, 256);
    chThdCreateStatic(adc_task_wa, sizeof(adc_task_wa), HIGHPRIO, adc_task, NULL);
}
