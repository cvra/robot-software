#include <ch.h>
#include <hal.h>

#define ENCODER_SPEED_FREQUENCY     1000.f

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

    uint32_t encoder_old;
    uint32_t encoder_new;

    encoder_new = encoder_get_primary();

    while (42) {
        encoder_old = encoder_new;
        encoder_new = encoder_get_primary();
        encoder_speed = encoder_speed * 0.9f
            + (int32_t)(encoder_new - encoder_old) / ENCODER_SPEED_FREQUENCY * 0.1f;

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
