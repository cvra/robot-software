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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_NAME                  "CVRA CAN IO board"

/*
 * Board oscillators-related settings.
 */
#define STM32_LSECLK                0U
#define STM32_LSEDRV                (3U << 3U)
#define STM32_HSECLK                16000000U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F302x8

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_PIN2                  2U
#define GPIOA_PIN3                  3U
#define GPIOA_LED                   4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_PIN9                  9U
#define GPIOA_PIN10                 10U
#define GPIOA_PIN11                 11U
#define GPIOA_PIN12                 12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_PIN0                  0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_SWO                   3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_PIN11                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_PIN13                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - GPIOA_PIN0                (input pullup).
 * PA1  - GPIOA_PIN1                (input pullup).
 * PA2  - GPIOA_PIN2                (input pullup).
 * PA3  - GPIOA_PIN3                (input pullup).
 * PA4  - GPIOA_LED                 (output low).
 * PA5  - GPIOA_PIN5                (input pullup).
 * PA6  - GPIOA_PIN6                (input pullup).
 * PA7  - GPIOA_PIN7                (input pullup).
 * PA8  - GPIOA_PIN8                (input pullup).
 * PA9  - GPIOA_PIN9                (input pullup).
 * PA10 - GPIOA_PIN10               (input pullup).
 * PA11 - GPIOA_PIN11               (input pullup).
 * PA12 - GPIOA_PIN12               (input pullup).
 * PA13 - GPIOA_SWDIO               (alternate 0).
 * PA14 - GPIOA_SWCLK               (alternate 0, pulldown).
 * PA15 - GPIOA_PIN15               (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) | \
                                     PIN_MODE_INPUT(GPIOA_PIN1) | \
                                     PIN_MODE_INPUT(GPIOA_PIN2) | \
                                     PIN_MODE_INPUT(GPIOA_PIN3) | \
                                     PIN_MODE_OUTPUT(GPIOA_LED) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_INPUT(GPIOA_PIN6) | \
                                     PIN_MODE_INPUT(GPIOA_PIN7) | \
                                     PIN_MODE_INPUT(GPIOA_PIN8) | \
                                     PIN_MODE_INPUT(GPIOA_PIN9) | \
                                     PIN_MODE_INPUT(GPIOA_PIN10) | \
                                     PIN_MODE_INPUT(GPIOA_PIN11) | \
                                     PIN_MODE_INPUT(GPIOA_PIN12) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_PIN0) | \
                                     PIN_OSPEED_100M(GPIOA_PIN1) | \
                                     PIN_OSPEED_100M(GPIOA_PIN2) | \
                                     PIN_OSPEED_100M(GPIOA_PIN3) | \
                                     PIN_OSPEED_2M(GPIOA_LED) | \
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PIN6) | \
                                     PIN_OSPEED_100M(GPIOA_PIN7) | \
                                     PIN_OSPEED_100M(GPIOA_PIN8) | \
                                     PIN_OSPEED_100M(GPIOA_PIN9) | \
                                     PIN_OSPEED_100M(GPIOA_PIN10) | \
                                     PIN_OSPEED_100M(GPIOA_PIN11) | \
                                     PIN_OSPEED_100M(GPIOA_PIN12) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_100M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_PIN0) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN1) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOA_LED) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN5) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN6) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN7) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN8) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN9) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN10) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN11) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN12) | \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_PIN0) | \
                                     PIN_ODR_LOW(GPIOA_PIN1) | \
                                     PIN_ODR_LOW(GPIOA_PIN2) | \
                                     PIN_ODR_LOW(GPIOA_PIN3) | \
                                     PIN_ODR_LOW(GPIOA_LED) | \
                                     PIN_ODR_LOW(GPIOA_PIN5) | \
                                     PIN_ODR_LOW(GPIOA_PIN6) | \
                                     PIN_ODR_LOW(GPIOA_PIN7) | \
                                     PIN_ODR_LOW(GPIOA_PIN8) | \
                                     PIN_ODR_LOW(GPIOA_PIN9) | \
                                     PIN_ODR_LOW(GPIOA_PIN10) | \
                                     PIN_ODR_LOW(GPIOA_PIN11) | \
                                     PIN_ODR_LOW(GPIOA_PIN12) | \
                                     PIN_ODR_LOW(GPIOA_SWDIO) | \
                                     PIN_ODR_LOW(GPIOA_SWCLK) | \
                                     PIN_ODR_LOW(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOA_LED, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 *
 * PB0  - GPIOB_PIN0                (input pullup).
 * PB1  - GPIOB_PIN1                (input pullup).
 * PB2  - GPIOB_PIN2                (input pullup).
 * PB3  - GPIOB_SWO                 (alternate 0).
 * PB4  - GPIOB_PIN4                (input pullup).
 * PB5  - GPIOB_PIN5                (input pullup).
 * PB6  - GPIOB_PIN6                (input pullup).
 * PB7  - GPIOB_PIN7                (input pullup).
 * PB8  - GPIOB_PIN8                (input floating).
 * PB9  - GPIOB_PIN9                (input floating).
 * PB10 - GPIOB_PIN10               (input floating).
 * PB11 - GPIOB_PIN11               (input floating).
 * PB12 - GPIOB_PIN12               (input floating).
 * PB13 - GPIOB_PIN13               (input floating).
 * PB14 - GPIOB_PIN14               (input floating).
 * PB15 - GPIOB_PIN15               (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) | \
                                     PIN_MODE_INPUT(GPIOB_PIN1) | \
                                     PIN_MODE_INPUT(GPIOB_PIN2) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) | \
                                     PIN_MODE_INPUT(GPIOB_PIN4) | \
                                     PIN_MODE_INPUT(GPIOB_PIN5) | \
                                     PIN_MODE_INPUT(GPIOB_PIN6) | \
                                     PIN_MODE_INPUT(GPIOB_PIN7) | \
                                     PIN_MODE_INPUT(GPIOB_PIN8) | \
                                     PIN_MODE_INPUT(GPIOB_PIN9) | \
                                     PIN_MODE_INPUT(GPIOB_PIN10) | \
                                     PIN_MODE_INPUT(GPIOB_PIN11) | \
                                     PIN_MODE_INPUT(GPIOB_PIN12) | \
                                     PIN_MODE_INPUT(GPIOB_PIN13) | \
                                     PIN_MODE_INPUT(GPIOB_PIN14) | \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_PIN0) | \
                                     PIN_OSPEED_100M(GPIOB_PIN1) | \
                                     PIN_OSPEED_100M(GPIOB_PIN2) | \
                                     PIN_OSPEED_100M(GPIOB_SWO) | \
                                     PIN_OSPEED_100M(GPIOB_PIN4) | \
                                     PIN_OSPEED_100M(GPIOB_PIN5) | \
                                     PIN_OSPEED_100M(GPIOB_PIN6) | \
                                     PIN_OSPEED_100M(GPIOB_PIN7) | \
                                     PIN_OSPEED_100M(GPIOB_PIN8) | \
                                     PIN_OSPEED_100M(GPIOB_PIN9) | \
                                     PIN_OSPEED_100M(GPIOB_PIN10) | \
                                     PIN_OSPEED_100M(GPIOB_PIN11) | \
                                     PIN_OSPEED_100M(GPIOB_PIN12) | \
                                     PIN_OSPEED_100M(GPIOB_PIN13) | \
                                     PIN_OSPEED_100M(GPIOB_PIN14) | \
                                     PIN_OSPEED_100M(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_PIN0) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN1) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SWO) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN4) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN5) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN6) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_PIN0) | \
                                     PIN_ODR_LOW(GPIOB_PIN1) | \
                                     PIN_ODR_LOW(GPIOB_PIN2) | \
                                     PIN_ODR_LOW(GPIOB_SWO) | \
                                     PIN_ODR_LOW(GPIOB_PIN4) | \
                                     PIN_ODR_LOW(GPIOB_PIN5) | \
                                     PIN_ODR_LOW(GPIOB_PIN6) | \
                                     PIN_ODR_LOW(GPIOB_PIN7) | \
                                     PIN_ODR_LOW(GPIOB_PIN8) | \
                                     PIN_ODR_LOW(GPIOB_PIN9) | \
                                     PIN_ODR_LOW(GPIOB_PIN10) | \
                                     PIN_ODR_LOW(GPIOB_PIN11) | \
                                     PIN_ODR_LOW(GPIOB_PIN12) | \
                                     PIN_ODR_LOW(GPIOB_PIN13) | \
                                     PIN_ODR_LOW(GPIOB_PIN14) | \
                                     PIN_ODR_LOW(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOB_SWO, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

/*
 * GPIOC setup:
 *
 * PC0  - GPIOC_PIN0                (input floating).
 * PC1  - GPIOC_PIN1                (input floating).
 * PC2  - GPIOC_PIN2                (input floating).
 * PC3  - GPIOC_PIN3                (input floating).
 * PC4  - GPIOC_PIN4                (input floating).
 * PC5  - GPIOC_PIN5                (input floating).
 * PC6  - GPIOC_PIN6                (input floating).
 * PC7  - GPIOC_PIN7                (input floating).
 * PC8  - GPIOC_PIN8                (input floating).
 * PC9  - GPIOC_PIN9                (input floating).
 * PC10 - GPIOC_PIN10               (input floating).
 * PC11 - GPIOC_PIN11               (input floating).
 * PC12 - GPIOC_PIN12               (input floating).
 * PC13 - GPIOC_PIN13               (input floating).
 * PC14 - GPIOC_PIN14               (input floating).
 * PC15 - GPIOC_PIN15               (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) | \
                                     PIN_MODE_INPUT(GPIOC_PIN1) | \
                                     PIN_MODE_INPUT(GPIOC_PIN2) | \
                                     PIN_MODE_INPUT(GPIOC_PIN3) | \
                                     PIN_MODE_INPUT(GPIOC_PIN4) | \
                                     PIN_MODE_INPUT(GPIOC_PIN5) | \
                                     PIN_MODE_INPUT(GPIOC_PIN6) | \
                                     PIN_MODE_INPUT(GPIOC_PIN7) | \
                                     PIN_MODE_INPUT(GPIOC_PIN8) | \
                                     PIN_MODE_INPUT(GPIOC_PIN9) | \
                                     PIN_MODE_INPUT(GPIOC_PIN10) | \
                                     PIN_MODE_INPUT(GPIOC_PIN11) | \
                                     PIN_MODE_INPUT(GPIOC_PIN12) | \
                                     PIN_MODE_INPUT(GPIOC_PIN13) | \
                                     PIN_MODE_INPUT(GPIOC_PIN14) | \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_PIN0) | \
                                     PIN_OSPEED_100M(GPIOC_PIN1) | \
                                     PIN_OSPEED_100M(GPIOC_PIN2) | \
                                     PIN_OSPEED_100M(GPIOC_PIN3) | \
                                     PIN_OSPEED_100M(GPIOC_PIN4) | \
                                     PIN_OSPEED_100M(GPIOC_PIN5) | \
                                     PIN_OSPEED_100M(GPIOC_PIN6) | \
                                     PIN_OSPEED_100M(GPIOC_PIN7) | \
                                     PIN_OSPEED_100M(GPIOC_PIN8) | \
                                     PIN_OSPEED_100M(GPIOC_PIN9) | \
                                     PIN_OSPEED_100M(GPIOC_PIN10) | \
                                     PIN_OSPEED_100M(GPIOC_PIN11) | \
                                     PIN_OSPEED_100M(GPIOC_PIN12) | \
                                     PIN_OSPEED_100M(GPIOC_PIN13) | \
                                     PIN_OSPEED_100M(GPIOC_PIN14) | \
                                     PIN_OSPEED_100M(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN0) | \
                                     PIN_ODR_LOW(GPIOC_PIN1) | \
                                     PIN_ODR_LOW(GPIOC_PIN2) | \
                                     PIN_ODR_LOW(GPIOC_PIN3) | \
                                     PIN_ODR_LOW(GPIOC_PIN4) | \
                                     PIN_ODR_LOW(GPIOC_PIN5) | \
                                     PIN_ODR_LOW(GPIOC_PIN6) | \
                                     PIN_ODR_LOW(GPIOC_PIN7) | \
                                     PIN_ODR_LOW(GPIOC_PIN8) | \
                                     PIN_ODR_LOW(GPIOC_PIN9) | \
                                     PIN_ODR_LOW(GPIOC_PIN10) | \
                                     PIN_ODR_LOW(GPIOC_PIN11) | \
                                     PIN_ODR_LOW(GPIOC_PIN12) | \
                                     PIN_ODR_LOW(GPIOC_PIN13) | \
                                     PIN_ODR_LOW(GPIOC_PIN14) | \
                                     PIN_ODR_LOW(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN7, 0))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0))

/*
 * GPIOD setup:
 *
 * PD0  - GPIOD_PIN0                (input floating).
 * PD1  - GPIOD_PIN1                (input floating).
 * PD2  - GPIOD_PIN2                (input floating).
 * PD3  - GPIOD_PIN3                (input floating).
 * PD4  - GPIOD_PIN4                (input floating).
 * PD5  - GPIOD_PIN5                (input floating).
 * PD6  - GPIOD_PIN6                (input floating).
 * PD7  - GPIOD_PIN7                (input floating).
 * PD8  - GPIOD_PIN8                (input floating).
 * PD9  - GPIOD_PIN9                (input floating).
 * PD10 - GPIOD_PIN10               (input floating).
 * PD11 - GPIOD_PIN11               (input floating).
 * PD12 - GPIOD_PIN12               (input floating).
 * PD13 - GPIOD_PIN13               (input floating).
 * PD14 - GPIOD_PIN14               (input floating).
 * PD15 - GPIOD_PIN15               (input floating).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) | \
                                     PIN_MODE_INPUT(GPIOD_PIN1) | \
                                     PIN_MODE_INPUT(GPIOD_PIN2) | \
                                     PIN_MODE_INPUT(GPIOD_PIN3) | \
                                     PIN_MODE_INPUT(GPIOD_PIN4) | \
                                     PIN_MODE_INPUT(GPIOD_PIN5) | \
                                     PIN_MODE_INPUT(GPIOD_PIN6) | \
                                     PIN_MODE_INPUT(GPIOD_PIN7) | \
                                     PIN_MODE_INPUT(GPIOD_PIN8) | \
                                     PIN_MODE_INPUT(GPIOD_PIN9) | \
                                     PIN_MODE_INPUT(GPIOD_PIN10) | \
                                     PIN_MODE_INPUT(GPIOD_PIN11) | \
                                     PIN_MODE_INPUT(GPIOD_PIN12) | \
                                     PIN_MODE_INPUT(GPIOD_PIN13) | \
                                     PIN_MODE_INPUT(GPIOD_PIN14) | \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_PIN0) | \
                                     PIN_OSPEED_100M(GPIOD_PIN1) | \
                                     PIN_OSPEED_100M(GPIOD_PIN2) | \
                                     PIN_OSPEED_100M(GPIOD_PIN3) | \
                                     PIN_OSPEED_100M(GPIOD_PIN4) | \
                                     PIN_OSPEED_100M(GPIOD_PIN5) | \
                                     PIN_OSPEED_100M(GPIOD_PIN6) | \
                                     PIN_OSPEED_100M(GPIOD_PIN7) | \
                                     PIN_OSPEED_100M(GPIOD_PIN8) | \
                                     PIN_OSPEED_100M(GPIOD_PIN9) | \
                                     PIN_OSPEED_100M(GPIOD_PIN10) | \
                                     PIN_OSPEED_100M(GPIOD_PIN11) | \
                                     PIN_OSPEED_100M(GPIOD_PIN12) | \
                                     PIN_OSPEED_100M(GPIOD_PIN13) | \
                                     PIN_OSPEED_100M(GPIOD_PIN14) | \
                                     PIN_OSPEED_100M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_PIN0) | \
                                     PIN_ODR_LOW(GPIOD_PIN1) | \
                                     PIN_ODR_LOW(GPIOD_PIN2) | \
                                     PIN_ODR_LOW(GPIOD_PIN3) | \
                                     PIN_ODR_LOW(GPIOD_PIN4) | \
                                     PIN_ODR_LOW(GPIOD_PIN5) | \
                                     PIN_ODR_LOW(GPIOD_PIN6) | \
                                     PIN_ODR_LOW(GPIOD_PIN7) | \
                                     PIN_ODR_LOW(GPIOD_PIN8) | \
                                     PIN_ODR_LOW(GPIOD_PIN9) | \
                                     PIN_ODR_LOW(GPIOD_PIN10) | \
                                     PIN_ODR_LOW(GPIOD_PIN11) | \
                                     PIN_ODR_LOW(GPIOD_PIN12) | \
                                     PIN_ODR_LOW(GPIOD_PIN13) | \
                                     PIN_ODR_LOW(GPIOD_PIN14) | \
                                     PIN_ODR_LOW(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input floating).
 * PE1  - PIN1                      (input floating).
 * PE2  - PIN2                      (input floating).
 * PE3  - PIN3                      (input floating).
 * PE4  - PIN4                      (input floating).
 * PE5  - PIN5                      (input floating).
 * PE6  - PIN6                      (input floating).
 * PE7  - PIN7                      (input floating).
 * PE8  - PIN8                      (input floating).
 * PE9  - PIN9                      (input floating).
 * PE10 - PIN10                     (input floating).
 * PE11 - PIN11                     (input floating).
 * PE12 - PIN12                     (input floating).
 * PE13 - PIN13                     (input floating).
 * PE14 - PIN14                     (input floating).
 * PE15 - PIN15                     (input floating).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) | \
                                     PIN_MODE_INPUT(GPIOE_PIN1) | \
                                     PIN_MODE_INPUT(GPIOE_PIN2) | \
                                     PIN_MODE_INPUT(GPIOE_PIN3) | \
                                     PIN_MODE_INPUT(GPIOE_PIN4) | \
                                     PIN_MODE_INPUT(GPIOE_PIN5) | \
                                     PIN_MODE_INPUT(GPIOE_PIN6) | \
                                     PIN_MODE_INPUT(GPIOE_PIN7) | \
                                     PIN_MODE_INPUT(GPIOE_PIN8) | \
                                     PIN_MODE_INPUT(GPIOE_PIN9) | \
                                     PIN_MODE_INPUT(GPIOE_PIN10) | \
                                     PIN_MODE_INPUT(GPIOE_PIN11) | \
                                     PIN_MODE_INPUT(GPIOE_PIN12) | \
                                     PIN_MODE_INPUT(GPIOE_PIN13) | \
                                     PIN_MODE_INPUT(GPIOE_PIN14) | \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_PIN0) | \
                                     PIN_OSPEED_100M(GPIOE_PIN1) | \
                                     PIN_OSPEED_100M(GPIOE_PIN2) | \
                                     PIN_OSPEED_100M(GPIOE_PIN3) | \
                                     PIN_OSPEED_100M(GPIOE_PIN4) | \
                                     PIN_OSPEED_100M(GPIOE_PIN5) | \
                                     PIN_OSPEED_100M(GPIOE_PIN6) | \
                                     PIN_OSPEED_100M(GPIOE_PIN7) | \
                                     PIN_OSPEED_100M(GPIOE_PIN8) | \
                                     PIN_OSPEED_100M(GPIOE_PIN9) | \
                                     PIN_OSPEED_100M(GPIOE_PIN10) | \
                                     PIN_OSPEED_100M(GPIOE_PIN11) | \
                                     PIN_OSPEED_100M(GPIOE_PIN12) | \
                                     PIN_OSPEED_100M(GPIOE_PIN13) | \
                                     PIN_OSPEED_100M(GPIOE_PIN14) | \
                                     PIN_OSPEED_100M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_LOW(GPIOE_PIN0) | \
                                     PIN_ODR_LOW(GPIOE_PIN1) | \
                                     PIN_ODR_LOW(GPIOE_PIN2) | \
                                     PIN_ODR_LOW(GPIOE_PIN3) | \
                                     PIN_ODR_LOW(GPIOE_PIN4) | \
                                     PIN_ODR_LOW(GPIOE_PIN5) | \
                                     PIN_ODR_LOW(GPIOE_PIN6) | \
                                     PIN_ODR_LOW(GPIOE_PIN7) | \
                                     PIN_ODR_LOW(GPIOE_PIN8) | \
                                     PIN_ODR_LOW(GPIOE_PIN9) | \
                                     PIN_ODR_LOW(GPIOE_PIN10) | \
                                     PIN_ODR_LOW(GPIOE_PIN11) | \
                                     PIN_ODR_LOW(GPIOE_PIN12) | \
                                     PIN_ODR_LOW(GPIOE_PIN13) | \
                                     PIN_ODR_LOW(GPIOE_PIN14) | \
                                     PIN_ODR_LOW(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input floating).
 * PF1  - PIN1                      (input floating).
 * PF2  - PIN2                      (input floating).
 * PF3  - PIN3                      (input floating).
 * PF4  - PIN4                      (input floating).
 * PF5  - PIN5                      (input floating).
 * PF6  - PIN6                      (input floating).
 * PF7  - PIN7                      (input floating).
 * PF8  - PIN8                      (input floating).
 * PF9  - PIN9                      (input floating).
 * PF10 - PIN10                     (input floating).
 * PF11 - PIN11                     (input floating).
 * PF12 - PIN12                     (input floating).
 * PF13 - PIN13                     (input floating).
 * PF14 - PIN14                     (input floating).
 * PF15 - PIN15                     (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) | \
                                     PIN_MODE_INPUT(GPIOF_PIN1) | \
                                     PIN_MODE_INPUT(GPIOF_PIN2) | \
                                     PIN_MODE_INPUT(GPIOF_PIN3) | \
                                     PIN_MODE_INPUT(GPIOF_PIN4) | \
                                     PIN_MODE_INPUT(GPIOF_PIN5) | \
                                     PIN_MODE_INPUT(GPIOF_PIN6) | \
                                     PIN_MODE_INPUT(GPIOF_PIN7) | \
                                     PIN_MODE_INPUT(GPIOF_PIN8) | \
                                     PIN_MODE_INPUT(GPIOF_PIN9) | \
                                     PIN_MODE_INPUT(GPIOF_PIN10) | \
                                     PIN_MODE_INPUT(GPIOF_PIN11) | \
                                     PIN_MODE_INPUT(GPIOF_PIN12) | \
                                     PIN_MODE_INPUT(GPIOF_PIN13) | \
                                     PIN_MODE_INPUT(GPIOF_PIN14) | \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_PIN0) | \
                                     PIN_OSPEED_100M(GPIOF_PIN1) | \
                                     PIN_OSPEED_100M(GPIOF_PIN2) | \
                                     PIN_OSPEED_100M(GPIOF_PIN3) | \
                                     PIN_OSPEED_100M(GPIOF_PIN4) | \
                                     PIN_OSPEED_100M(GPIOF_PIN5) | \
                                     PIN_OSPEED_100M(GPIOF_PIN6) | \
                                     PIN_OSPEED_100M(GPIOF_PIN7) | \
                                     PIN_OSPEED_100M(GPIOF_PIN8) | \
                                     PIN_OSPEED_100M(GPIOF_PIN9) | \
                                     PIN_OSPEED_100M(GPIOF_PIN10) | \
                                     PIN_OSPEED_100M(GPIOF_PIN11) | \
                                     PIN_OSPEED_100M(GPIOF_PIN12) | \
                                     PIN_OSPEED_100M(GPIOF_PIN13) | \
                                     PIN_OSPEED_100M(GPIOF_PIN14) | \
                                     PIN_OSPEED_100M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_PIN0) | \
                                     PIN_ODR_LOW(GPIOF_PIN1) | \
                                     PIN_ODR_LOW(GPIOF_PIN2) | \
                                     PIN_ODR_LOW(GPIOF_PIN3) | \
                                     PIN_ODR_LOW(GPIOF_PIN4) | \
                                     PIN_ODR_LOW(GPIOF_PIN5) | \
                                     PIN_ODR_LOW(GPIOF_PIN6) | \
                                     PIN_ODR_LOW(GPIOF_PIN7) | \
                                     PIN_ODR_LOW(GPIOF_PIN8) | \
                                     PIN_ODR_LOW(GPIOF_PIN9) | \
                                     PIN_ODR_LOW(GPIOF_PIN10) | \
                                     PIN_ODR_LOW(GPIOF_PIN11) | \
                                     PIN_ODR_LOW(GPIOF_PIN12) | \
                                     PIN_ODR_LOW(GPIOF_PIN13) | \
                                     PIN_ODR_LOW(GPIOF_PIN14) | \
                                     PIN_ODR_LOW(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
