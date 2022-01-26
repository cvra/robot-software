#include <ch.h>
#include <hal.h>

#define MAX_16BIT 65535
#define MAX_16BIT_DIV2 32767

static void setup_timer(stm32_tim_t* tmr);
static uint32_t encoder_get_left_raw(void);
static uint32_t encoder_get_right_raw(void);
static int32_t encoder_tick_diff(uint32_t enc_old, uint32_t enc_new);

static uint32_t left_encoder_prev, right_encoder_prev;
static int32_t left_encoder_value, right_encoder_value;

int32_t encoder_get_left()
{
    uint32_t left_encoder = encoder_get_left_raw();
    left_encoder_value += encoder_tick_diff(left_encoder_prev, left_encoder);
    left_encoder_prev = left_encoder;
    return left_encoder_value;
}

int32_t encoder_get_right()
{
    uint32_t right_encoder = encoder_get_right_raw();
    right_encoder_value += encoder_tick_diff(right_encoder_prev, right_encoder);
    right_encoder_prev = right_encoder;
    return right_encoder_value;
}

void encoder_init(void)
{
    rccEnableTIM4(FALSE); // enable timer 4 clock
    rccResetTIM4();
    setup_timer(STM32_TIM4);

    palSetPadMode(GPIOB, GPIOB_GPIO_A, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOB, GPIOB_GPIO_B, PAL_MODE_ALTERNATE(2));

    rccEnableTIM3(FALSE); // enable timer 3 clock
    rccResetTIM3();
    setup_timer(STM32_TIM3);

    left_encoder_prev = encoder_get_left_raw();
    right_encoder_prev = encoder_get_right_raw();
    left_encoder_value = left_encoder_prev;
    right_encoder_value = right_encoder_prev;
}

static void setup_timer(stm32_tim_t* tim)
{
    tim->CR2 = 0;
    tim->PSC = 0; // Prescaler value.
    tim->SR = 0; // Clear pending IRQs.
    tim->DIER = 0; // DMA-related DIER bits.
    tim->SMCR = STM32_TIM_SMCR_SMS(3); // count on both edges
    tim->CCMR1 = STM32_TIM_CCMR1_CC1S(1); // CC1 channel is input, IC1 is mapped on TI1
    tim->CCMR1 |= STM32_TIM_CCMR1_CC2S(1); // CC2 channel is input, IC2 is mapped on TI2
    /* enable filtering, clock prescaler 32, 8 samples */
    tim->CCMR1 |= STM32_TIM_CCMR1_IC1F(0xf);
    tim->CCMR1 |= STM32_TIM_CCMR1_IC2F(0xf);
    tim->CCER = 0;
    tim->ARR = 0xFFFF;
    tim->CR1 = 1; // start
}

static uint32_t encoder_get_left_raw(void)
{
    return STM32_TIM4->CNT;
}

static uint32_t encoder_get_right_raw(void)
{
    return STM32_TIM3->CNT;
}

static int32_t encoder_tick_diff(uint32_t enc_old, uint32_t enc_new)
{
    if (enc_old < enc_new) {
        if (enc_new - enc_old <= MAX_16BIT_DIV2) {
            return enc_new - enc_old;
        } else {
            return -(MAX_16BIT + 1 - (enc_new - enc_old));
        }
    } else if (enc_old > enc_new) {
        if (enc_old - enc_new <= MAX_16BIT_DIV2) {
            return -(enc_old - enc_new);
        } else {
            return MAX_16BIT + 1 - (enc_old - enc_new);
        }
    } else {
        return 0;
    }
}
