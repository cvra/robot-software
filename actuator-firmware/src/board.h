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

/* clang-format off */

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_NAME                  "CVRA Actuator board"

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
#define GPIOA_GPIO_ANALOG0          0U
#define GPIOA_GPIO_ANALOG1          1U
#define GPIOA_PIN2                  2U
#define GPIOA_PIN3                  3U
#define GPIOA_LED                   4U
#define GPIOA_PIN5                  5U
#define GPIOA_PWM0                  6U
#define GPIOA_PWM1                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_PWM2                  9U
#define GPIOA_PWM3                  10U
#define GPIOA_CAN_RX                11U
#define GPIOA_CAN_TX                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_CAN_DIS               0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_SPI_SCK               3U
#define GPIOB_SPI_MISO              4U
#define GPIOB_SPI_MOSI              5U
#define GPIOB_DEBUG_TX              6U
#define GPIOB_DEBUG_RX              7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_PIN11                 11U
#define GPIOB_PIN12                 12U
#define GPIOB_PIN13                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

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
 * PA0  - GPIOA_GPIO_ANALOG0        (analog floating).
 * PA1  - GPIOA_GPIO_ANALOG1        (analog floating).
 * PA2  - GPIOA_PIN2                (input pullup).
 * PA3  - GPIOA_PIN3                (input pullup).
 * PA4  - GPIOA_LED                 (output low).
 * PA5  - GPIOA_PIN5                (input pullup).
 * PA6  - GPIOA_PWM0                (alternate 1 (TIM16_CH1)).
 * PA7  - GPIOA_PWM1                (alternate 1 (TIM17_CH1)).
 * PA8  - GPIOA_PIN8                (input pullup).
 * PA9  - GPIOA_PWM2                (alternate 6 (TIM1_CH2)).
 * PA10 - GPIOA_PWM3                (alternate 6 (TIM1_CH3)).
 * PA11 - GPIOA_CAN_RX              (alternate 9).
 * PA12 - GPIOA_CAN_TX              (alternate 9).
 * PA13 - GPIOA_SWDIO               (alternate 0).
 * PA14 - GPIOA_SWCLK               (alternate 0, pulldown).
 * PA15 - GPIOA_PIN15               (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_GPIO_ANALOG0) | \
                                     PIN_MODE_ANALOG(GPIOA_GPIO_ANALOG1) | \
                                     PIN_MODE_INPUT(GPIOA_PIN2) | \
                                     PIN_MODE_INPUT(GPIOA_PIN3) | \
                                     PIN_MODE_OUTPUT(GPIOA_LED) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM0) | \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM1) | \
                                     PIN_MODE_INPUT(GPIOA_PIN8) | \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM2) | \
                                     PIN_MODE_ALTERNATE(GPIOA_PWM3) | \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_CAN_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_GPIO_ANALOG0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIO_ANALOG1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PWM3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_GPIO_ANALOG0) | \
                                     PIN_OSPEED_100M(GPIOA_GPIO_ANALOG1) | \
                                     PIN_OSPEED_100M(GPIOA_PIN2) | \
                                     PIN_OSPEED_100M(GPIOA_PIN3) | \
                                     PIN_OSPEED_2M(GPIOA_LED) | \
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PWM0) | \
                                     PIN_OSPEED_100M(GPIOA_PWM1) | \
                                     PIN_OSPEED_100M(GPIOA_PIN8) | \
                                     PIN_OSPEED_100M(GPIOA_PWM2) | \
                                     PIN_OSPEED_100M(GPIOA_PWM3) | \
                                     PIN_OSPEED_100M(GPIOA_CAN_RX) | \
                                     PIN_OSPEED_100M(GPIOA_CAN_TX) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_100M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_GPIO_ANALOG0) | \
                                     PIN_PUPDR_FLOATING(GPIOA_GPIO_ANALOG1) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOA_LED) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN5) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PWM0) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PWM1) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN8) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PWM2) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PWM3) | \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN_RX) | \
                                     PIN_PUPDR_PULLUP(GPIOA_CAN_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_GPIO_ANALOG0) | \
                                     PIN_ODR_LOW(GPIOA_GPIO_ANALOG1) | \
                                     PIN_ODR_LOW(GPIOA_PIN2) | \
                                     PIN_ODR_LOW(GPIOA_PIN3) | \
                                     PIN_ODR_LOW(GPIOA_LED) | \
                                     PIN_ODR_LOW(GPIOA_PIN5) | \
                                     PIN_ODR_LOW(GPIOA_PWM0) | \
                                     PIN_ODR_LOW(GPIOA_PWM1) | \
                                     PIN_ODR_LOW(GPIOA_PIN8) | \
                                     PIN_ODR_LOW(GPIOA_PWM2) | \
                                     PIN_ODR_LOW(GPIOA_PWM3) | \
                                     PIN_ODR_LOW(GPIOA_CAN_RX) | \
                                     PIN_ODR_LOW(GPIOA_CAN_TX) | \
                                     PIN_ODR_LOW(GPIOA_SWDIO) | \
                                     PIN_ODR_LOW(GPIOA_SWCLK) | \
                                     PIN_ODR_LOW(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_GPIO_ANALOG0, 0) | \
                                     PIN_AFIO_AF(GPIOA_GPIO_ANALOG1, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOA_LED, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PWM0, 1) | \
                                     PIN_AFIO_AF(GPIOA_PWM1, 1))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOA_PWM2, 6) | \
                                     PIN_AFIO_AF(GPIOA_PWM3, 6) | \
                                     PIN_AFIO_AF(GPIOA_CAN_RX, 9) | \
                                     PIN_AFIO_AF(GPIOA_CAN_TX, 9) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 *
 * PB0  - GPIOB_CAN_DIS             (output low).
 * PB1  - GPIOB_PIN1                (input pullup).
 * PB2  - GPIOB_PIN2                (input pullup).
 * PB3  - GPIOB_SPI_SCK             (alternate 6, SPI3).
 * PB4  - GPIOB_SPI_MISO            (alternate 6, SPI3).
 * PB5  - GPIOB_SPI_MOSI            (alternate 6, SPI3).
 * PB6  - GPIOB_DEBUG_TX            (alternate 7 (UART1 TX)).
 * PB7  - GPIOB_DEBUG_RX            (alternate 7 (UART1 RX)).
 * PB8  - GPIOB_PIN8                (input floating).
 * PB9  - GPIOB_PIN9                (input floating).
 * PB10 - GPIOB_PIN10               (input floating).
 * PB11 - GPIOB_PIN11               (input floating).
 * PB12 - GPIOB_PIN12               (input floating).
 * PB13 - GPIOB_PIN13               (input floating).
 * PB14 - GPIOB_PIN14               (input floating).
 * PB15 - GPIOB_PIN15               (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_CAN_DIS) | \
                                     PIN_MODE_INPUT(GPIOB_PIN1) | \
                                     PIN_MODE_INPUT(GPIOB_PIN2) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_SCK) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MOSI) | \
                                     PIN_MODE_ALTERNATE(GPIOB_DEBUG_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOB_DEBUG_RX) | \
                                     PIN_MODE_INPUT(GPIOB_PIN8) | \
                                     PIN_MODE_INPUT(GPIOB_PIN9) | \
                                     PIN_MODE_INPUT(GPIOB_PIN10) | \
                                     PIN_MODE_INPUT(GPIOB_PIN11) | \
                                     PIN_MODE_INPUT(GPIOB_PIN12) | \
                                     PIN_MODE_INPUT(GPIOB_PIN13) | \
                                     PIN_MODE_INPUT(GPIOB_PIN14) | \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_CAN_DIS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_SCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MOSI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DEBUG_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_DEBUG_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_CAN_DIS) | \
                                     PIN_OSPEED_100M(GPIOB_PIN1) | \
                                     PIN_OSPEED_100M(GPIOB_PIN2) | \
                                     PIN_OSPEED_100M(GPIOB_SPI_SCK) | \
                                     PIN_OSPEED_100M(GPIOB_SPI_MISO) | \
                                     PIN_OSPEED_100M(GPIOB_SPI_MOSI) | \
                                     PIN_OSPEED_100M(GPIOB_DEBUG_TX) | \
                                     PIN_OSPEED_100M(GPIOB_DEBUG_RX) | \
                                     PIN_OSPEED_100M(GPIOB_PIN8) | \
                                     PIN_OSPEED_100M(GPIOB_PIN9) | \
                                     PIN_OSPEED_100M(GPIOB_PIN10) | \
                                     PIN_OSPEED_100M(GPIOB_PIN11) | \
                                     PIN_OSPEED_100M(GPIOB_PIN12) | \
                                     PIN_OSPEED_100M(GPIOB_PIN13) | \
                                     PIN_OSPEED_100M(GPIOB_PIN14) | \
                                     PIN_OSPEED_100M(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_CAN_DIS) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN1) | \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_SCK) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_MISO) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_MOSI) | \
                                     PIN_PUPDR_PULLUP(GPIOB_DEBUG_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOB_DEBUG_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_CAN_DIS) | \
                                     PIN_ODR_LOW(GPIOB_PIN1) | \
                                     PIN_ODR_LOW(GPIOB_PIN2) | \
                                     PIN_ODR_LOW(GPIOB_SPI_SCK) | \
                                     PIN_ODR_LOW(GPIOB_SPI_MISO) | \
                                     PIN_ODR_LOW(GPIOB_SPI_MOSI) | \
                                     PIN_ODR_LOW(GPIOB_DEBUG_TX) | \
                                     PIN_ODR_LOW(GPIOB_DEBUG_RX) | \
                                     PIN_ODR_LOW(GPIOB_PIN8) | \
                                     PIN_ODR_LOW(GPIOB_PIN9) | \
                                     PIN_ODR_LOW(GPIOB_PIN10) | \
                                     PIN_ODR_LOW(GPIOB_PIN11) | \
                                     PIN_ODR_LOW(GPIOB_PIN12) | \
                                     PIN_ODR_LOW(GPIOB_PIN13) | \
                                     PIN_ODR_LOW(GPIOB_PIN14) | \
                                     PIN_ODR_LOW(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_CAN_DIS, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOB_SPI_SCK, 6) | \
                                     PIN_AFIO_AF(GPIOB_SPI_MISO, 6) | \
                                     PIN_AFIO_AF(GPIOB_SPI_MOSI, 6) | \
                                     PIN_AFIO_AF(GPIOB_DEBUG_TX, 7) | \
                                     PIN_AFIO_AF(GPIOB_DEBUG_RX, 7))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

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
/* clang-format on */
