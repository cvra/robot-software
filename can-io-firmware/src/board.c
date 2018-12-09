/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
 */

#include "hal.h"

/**
 * @brief   Type of STM32 GPIO port setup.
 */
typedef struct {
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t odr;
    uint32_t afrl;
    uint32_t afrh;
} gpio_setup_t;

/**
 * @brief   Type of STM32 GPIO initialization data.
 */
typedef struct {
#if STM32_HAS_GPIOA || defined(__DOXYGEN__)
    gpio_setup_t PAData;
#endif
#if STM32_HAS_GPIOB || defined(__DOXYGEN__)
    gpio_setup_t PBData;
#endif
} gpio_config_t;

/**
 * @brief   STM32 GPIO static initialization data.
 */
static const gpio_config_t gpio_default_config = {
#if STM32_HAS_GPIOA
    {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
     VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH},
#endif
#if STM32_HAS_GPIOB
    {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
     VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH},
#endif
};

static void gpio_init(stm32_gpio_t* gpiop, const gpio_setup_t* config)
{
    gpiop->OTYPER = config->otyper;
    gpiop->OSPEEDR = config->ospeedr;
    gpiop->PUPDR = config->pupdr;
    gpiop->ODR = config->odr;
    gpiop->AFRL = config->afrl;
    gpiop->AFRH = config->afrh;
    gpiop->MODER = config->moder;
}

static void stm32_gpio_init(void)
{
    /* Enabling GPIO-related clocks, the mask comes from the
     registry header file.*/
    rccResetAHB(STM32_GPIO_EN_MASK);
    rccEnableAHB(STM32_GPIO_EN_MASK, true);

    /* Initializing all the defined GPIO ports.*/
#if STM32_HAS_GPIOA
    gpio_init(GPIOA, &gpio_default_config.PAData);
#endif
#if STM32_HAS_GPIOB
    gpio_init(GPIOB, &gpio_default_config.PBData);
#endif
}

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void)
{
    stm32_gpio_init();
    stm32_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void)
{
}
