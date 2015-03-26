#include <ch.h>
#include <hal.h>
#include <filter/iir.h>

#define ENCODER_SPEED_FREQUENCY     500.f

static float encoder_speed;


uint32_t encoder_get_primary(void)
{
    return STM32_TIM4->CNT;
}

uint32_t encoder_get_secondary(void)
{
    return STM32_TIM3->CNT;
}

float encoder_get_speed(void)
{
    return encoder_speed;
}

static THD_FUNCTION(encoder_speed_task, arg)
{
    (void)arg;
    chRegSetThreadName("Encoder speed");

    filter_iir_t speed_filter;
    // scipy.signal.iirdesign(0.15, 0.25, 10, 40)
    // const float b[] = {0.00963341, -0.00442803, -0.00442803, 0.00963341};
    // const float a[] = {-2.72497916,  2.63867989, -0.90328997};
    const float b[] = {0.01, 0.0};
    const float a[] = {-0.99};
    float speed_filter_buffer[3];

    filter_iir_init(&speed_filter, b, a, 1, speed_filter_buffer);

    uint16_t encoder_old;
    uint16_t encoder_new;
    systime_t delta_time = 0;       // in 0.1 ms steps :(
    systime_t time = chVTGetSystemTimeX();

    encoder_new = encoder_get_primary();

    while (42) {
        encoder_old = encoder_new;
        encoder_new = encoder_get_primary();
        delta_time = chVTGetSystemTimeX() - time;
        delta_time = 20;
        time += delta_time;

        // encoder_speed = filter_iir_apply(&speed_filter,
        //             (float)((int16_t)(encoder_new - encoder_old))
        //             * 10000 / (uint32_t)delta_time);
        encoder_speed = (float)((int16_t)(encoder_new - encoder_old))
                        * 10000 / (uint32_t)delta_time;

        chThdSleepMicroseconds(1000000.f / ENCODER_SPEED_FREQUENCY);
    }

    return 0;
}

void encoder_init_primary(void)
{
    rccEnableTIM4(FALSE);                           // enable timer 4 clock
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

    static THD_WORKING_AREA(encoder_speed_wa, 256);
    chThdCreateStatic(encoder_speed_wa, sizeof(encoder_speed_wa), LOWPRIO, encoder_speed_task, NULL);
}

void encoder_init_secondary(void)
{
    palSetPadMode(GPIOB, GPIOB_GPIO_A, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOB, GPIOB_GPIO_B, PAL_MODE_ALTERNATE(2));

    rccEnableTIM3(FALSE);                           // enable timer 3 clock
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
}
