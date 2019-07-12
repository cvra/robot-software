#include <ch.h>
#include <hal.h>
#include <msgbus/messagebus.h>
#include "main.h"
#include "priorities.h"
#include "encoder.h"
#include "protobuf/encoders.pb.h"

#define ENCODER_STACKSIZE 512

#define MAX_16BIT 65535
#define MAX_16BIT_DIV2 32767

static void setup_timer(stm32_tim_t* tmr);

uint32_t encoder_get_left(void)
{
    return STM32_TIM4->CNT;
}

uint32_t encoder_get_right(void)
{
    return STM32_TIM3->CNT;
}

int encoder_tick_diff(uint32_t enc_old, uint32_t enc_new)
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

static THD_FUNCTION(encoders_thd, arg)
{
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    /* Setup and advertise encoders topic */
    static TOPIC_DECL(encoders_topic, WheelEncodersPulse);

    messagebus_advertise_topic(&bus, &encoders_topic.topic, "/encoders");

    /* Configure encoder timers */
    rccEnableTIM4(FALSE);
    rccResetTIM4();
    setup_timer(STM32_TIM4);

    rccEnableTIM3(FALSE);
    rccResetTIM3();
    setup_timer(STM32_TIM3);

    uint32_t left_old, right_old;
    WheelEncodersPulse values = WheelEncodersPulse_init_zero;

    left_old = encoder_get_left();
    right_old = encoder_get_right();

    while (1) {
        uint32_t left, right;

        left = encoder_get_left();
        right = encoder_get_right();

        values.left += encoder_tick_diff(left_old, left);
        values.right += encoder_tick_diff(right_old, right);

        messagebus_topic_publish(&encoders_topic.topic, &values, sizeof(values));

        left_old = left;
        right_old = right;
        chThdSleepMilliseconds(10);
    }
}

static void setup_timer(stm32_tim_t* tmr)
{
    tmr->CR2 = 0;
    tmr->PSC = 0; // Prescaler value.
    tmr->SR = 0; // Clear pending IRQs.
    tmr->DIER = 0; // DMA-related DIER bits.
    tmr->SMCR = STM32_TIM_SMCR_SMS(3); // count on both edges
    tmr->CCMR1 = STM32_TIM_CCMR1_CC1S(1); // CC1 channel is input, IC1 is mapped on TI1
    tmr->CCMR1 |= STM32_TIM_CCMR1_CC2S(1); // CC2 channel is input, IC2 is mapped on TI2
    tmr->CCMR1 |= STM32_TIM_CCMR1_IC1F(3); // Activate some input filtering
    tmr->CCER = 0;
    tmr->ARR = 0xFFFF;
    tmr->CR1 = 1; // start
}

void encoder_start(void)
{
    static THD_WORKING_AREA(encoders_thd_wa, ENCODER_STACKSIZE);
    chThdCreateStatic(encoders_thd_wa, sizeof(encoders_thd_wa), ENCODER_PRIO, encoders_thd, NULL);
}
