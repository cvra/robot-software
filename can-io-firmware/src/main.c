#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "bootloader_config.h"
#include "error/error.h"
#include "debug.h"

THD_FUNCTION(blinker, arg)
{
    (void) arg;
    while (1) {
        palSetPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(100);
    }
}

static void blinker_start(void)
{
    static THD_WORKING_AREA(blinker_wa, 256);
    chThdCreateStatic(blinker_wa, sizeof(blinker_wa), LOWPRIO, blinker, NULL);
}


int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    blinker_start();

    bootloader_config_t cfg;

    rccEnableAPB2(RCC_APB2ENR_TIM16EN, true);

    TIM16->CR1 = STM32_TIM_CR1_CEN | STM32_TIM_CR1_ARPE;
    TIM16->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE;
    TIM16->CCER = STM32_TIM_CCER_CC1E;
    TIM16->EGR = STM32_TIM_EGR_UG;
    TIM16->BDTR = STM32_TIM_BDTR_MOE;
    TIM16->CCR1 = 12000;
    TIM16->ARR = 20000;

    if (!config_get(&cfg)) {
        chSysHalt("Cannot load config");
    }

    NOTICE("Board name=\"%s\", ID=%d", cfg.board_name, cfg.ID);

    // Never returns
    uavcan_start(cfg.ID, cfg.board_name);

    return 0;
}
