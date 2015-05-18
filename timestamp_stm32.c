#include <stdlib.h>
#include <stdint.h>

static volatile uint32_t time_us_low;
static volatile uint32_t time_us_high;

// ChibiOS specific begin
#include <ch.h>
#include <hal.h>
#include "timestamp.h"

// settings
#include <timestamp_stm32_settings.h>

static inline uint32_t timer_read(void)
{
    uint32_t tim_reg = TIMER_REG->CNT;
    while (TIMER_REG->SR & STM32_TIM_SR_UIF); // make sure there is no interrupt pending TODO won't work in interrupts
    return tim_reg;
}


CH_FAST_IRQ_HANDLER(TIMER_IRQ_NAME)
{
    TIMER_REG->SR &= ~STM32_TIM_SR_UIF; // clear interrupt flag
    time_us_low += COUNTER_MAX;
    // todo
}

void timestamp_stm32_init(void)
{
    time_us_low = 0;
    time_us_high = 0;
    RCC_EN();
    RCC_RESET();
    nvicEnableVector(NVIC_NB, INTERRUPT_PRIO);
    TIMER_REG->ARR = COUNTER_MAX;
    TIMER_REG->PSC = PRESCALER;
    TIMER_REG->DIER = STM32_TIM_DIER_UIE; // enable update interrupt
    TIMER_REG->CR1 |= STM32_TIM_CR1_CEN; // enable timer
}
// ChibiOS specific end


timestamp_t timestamp_get()
{
    uint32_t t1 = time_us_low;
    uint32_t tim = timer_read();
    uint32_t t2 = time_us_low;
    if (t2 != t1) {
        return t2;
    }
    return t1 + tim;
}

ltimestamp_t ltimestamp_get()
{
    uint32_t t1 = time_us_low;
    uint32_t th = time_us_high;
    uint32_t tim = timer_read();
    uint32_t t2 = time_us_low;
    if (t2 != t1) {
        if (t2 < t1) { // time_us_low overflow
            return ((uint64_t)(th + 1) << 32) + t2;
        } else {
            return ((uint64_t)th << 32) + t2;
        }
    }
    return ((uint64_t)th << 32) + t1 + tim;
}

// test to make sure timestamps are monotonic
bool _timestamp_test(void)
{
    timestamp_t t = timestamp_get();
    while (1) {
        timestamp_t t_now = timestamp_get();
        if (timestamp_duration_us(t, t_now) < 0) {
            return false;
        }
        t = t_now;
    }
}
