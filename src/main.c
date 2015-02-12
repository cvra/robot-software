#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <blocking_uart_driver.h>
#include "motor_pwm.h"
#include "control.h"

BlockingUARTDriver blocking_uart_stream;
BaseSequentialStream* stderr = (BaseSequentialStream*)&blocking_uart_stream;
BaseSequentialStream* stdout;


static THD_WORKING_AREA(stream_task_wa, 256);
static THD_FUNCTION(stream_task, arg)
{
    (void)arg;
    chRegSetThreadName("print data");
    while(42){
        chprintf(stdout, "%2.3f A    %2.3f V\n", control_get_motor_current(), control_get_motor_voltage());
        chThdSleepMilliseconds(300);
    }
    return 0;
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



void panic_hook(const char* reason)
{
    palClearPad(GPIOA, GPIOA_LED);      // turn on LED (active low)
    blocking_uart_init(&blocking_uart_stream, USART3, 115200);
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


    sdStart(&SD3, NULL);
    stdout = (BaseSequentialStream*)&SD3;

    motor_pwm_setup();
    motor_pwm_enable();
    motor_pwm_set(0.0);

    control_start();

    chprintf(stdout, "boot\n");

    chThdCreateStatic(stream_task_wa, sizeof(stream_task_wa), LOWPRIO, stream_task, NULL);
    chThdCreateStatic(quad_task_wa, sizeof(quad_task_wa), LOWPRIO, quad_task, NULL);
    chThdCreateStatic(ext_quad_task_wa, sizeof(ext_quad_task_wa), LOWPRIO, ext_quad_task, NULL);

    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(1000);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(1000);
    }
}
