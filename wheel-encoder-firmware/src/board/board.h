/* clang-format off */
#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for CVRA motor control board.
 */

/*
 * Board identifier.
 */
#define BOARD_CVRA_MOTOR_CONTROL
#define BOARD_NAME                  "CVRA motor control"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0
#endif

#define STM32_LSEDRV                (3 << 3)

#define STM32_HSECLK                16000000

/*
 * MCU type as defined in the ST header.
 */
#define STM32F303xC

/*
 * IO pins assignments.
 */
#define GPIOA_GPIO_ADC              0
#define GPIOA_LED                   1 // active low
#define GPIOA_NC_2                  2
#define GPIOA_NC_3                  3
#define GPIOA_CAN_SPEED             4 // low = CAN enabled
#define GPIOA_ADC_I_MOT             5
#define GPIOA_ADC_V_BAT             6
#define GPIOA_NC_7                  7
#define GPIOA_MOTOR_PWM_A           8
#define GPIOA_MOTOR_PWM_B           9
#define GPIOA_MOTOR_EN_A            10
#define GPIOA_MOTOR_EN_B            11
#define GPIOA_GPIO_I                12
#define GPIOA_SWDIO                 13
#define GPIOA_SWCLK                 14
#define GPIOA_NC_15                 15

#define GPIOB_NC_0                  0
#define GPIOB_NC_1                  1
#define GPIOB_NC_2                  2
#define GPIOB_NC_3                  3
#define GPIOB_GPIO_A                4
#define GPIOB_GPIO_B                5
#define GPIOB_QUAD_ENC_A            6
#define GPIOB_QUAD_ENC_B            7
#define GPIOB_CAN_RX                8
#define GPIOB_CAN_TX                9
#define GPIOB_USART3_TX             10
#define GPIOB_USART3_RX             11
#define GPIOB_NC_12                 12
#define GPIOB_NC_13                 13
#define GPIOB_NC_14                 14
#define GPIOB_NC_15                 15

#define GPIOC_NC_13                 13
#define GPIOC_NC_14                 14
#define GPIOC_NC_15                 15

#define GPIOF_OSC_IN                0
#define GPIOF_OSC_OUT               1


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_10M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 *
 * PA0  - GPIOA_GPIO_ADC            (input floating). (ADC1_IN1)
 * PA1  - GPIOA_LED                 (output pushpull, high).
 * PA2  - GPIOA_NC_2                (input pulldown).
 * PA3  - GPIOA_NC_3                (input pulldown).
 * PA4  - GPIOA_CAN_SPEED           (output pushpull, high).
 * PA5  - GPIOA_ADC_I_MOT           (analog (ADC2_IN2)).
 * PA6  - GPIOA_ADC_V_BAT           (analog (ADC2_IN3)).
 * PA7  - GPIOA_NC_7                (input pulldown).
 * PA8  - GPIOA_MOTOR_PWM_A         (alternate 6 (TIM1_CH1)).
 * PA9  - GPIOA_MOTOR_PWM_B         (alternate 6 (TIM1_CH2)).
 * PA10 - GPIOA_MOTOR_EN_A          (output pushpull, low).
 * PA11 - GPIOA_MOTOR_EN_B          (output pushpull, low).
 * PA12 - GPIOA_GPIO_I              (input floating).
 * PA13 - GPIOA_SWDIO               (alternate 0).
 * PA14 - GPIOA_SWCLK               (alternate 0).
 * PA15 - GPIOA_NC_15               (input pulldown). // alternate 0?
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_GPIO_ADC) |       \
                                     PIN_MODE_OUTPUT(GPIOA_LED) |           \
                                     PIN_MODE_INPUT(GPIOA_NC_2) |           \
                                     PIN_MODE_INPUT(GPIOA_NC_3) |           \
                                     PIN_MODE_OUTPUT(GPIOA_CAN_SPEED) |     \
                                     PIN_MODE_ANALOG(GPIOA_ADC_I_MOT) |     \
                                     PIN_MODE_ANALOG(GPIOA_ADC_V_BAT) |     \
                                     PIN_MODE_INPUT(GPIOA_NC_7) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_MOTOR_PWM_A) |\
                                     PIN_MODE_ALTERNATE(GPIOA_MOTOR_PWM_B) |\
                                     PIN_MODE_OUTPUT(GPIOA_MOTOR_EN_A) |    \
                                     PIN_MODE_OUTPUT(GPIOA_MOTOR_EN_B) |    \
                                     PIN_MODE_INPUT(GPIOA_GPIO_I) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_NC_15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_GPIO_ADC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_NC_2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_NC_3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_CAN_SPEED) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC_I_MOT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC_V_BAT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_NC_7) |       \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_MOTOR_PWM_A) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOA_MOTOR_PWM_B) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOA_MOTOR_EN_A) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_MOTOR_EN_B) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_GPIO_I) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_NC_15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_50M(GPIOA_GPIO_ADC) |       \
                                     PIN_OSPEED_50M(GPIOA_LED) |            \
                                     PIN_OSPEED_50M(GPIOA_NC_2) |           \
                                     PIN_OSPEED_50M(GPIOA_NC_3) |           \
                                     PIN_OSPEED_50M(GPIOA_CAN_SPEED) |      \
                                     PIN_OSPEED_50M(GPIOA_ADC_I_MOT) |      \
                                     PIN_OSPEED_50M(GPIOA_ADC_V_BAT) |      \
                                     PIN_OSPEED_50M(GPIOA_NC_7) |           \
                                     PIN_OSPEED_50M(GPIOA_MOTOR_PWM_A) |    \
                                     PIN_OSPEED_50M(GPIOA_MOTOR_PWM_B) |    \
                                     PIN_OSPEED_50M(GPIOA_MOTOR_EN_A) |     \
                                     PIN_OSPEED_50M(GPIOA_MOTOR_EN_B) |     \
                                     PIN_OSPEED_50M(GPIOA_GPIO_I) |         \
                                     PIN_OSPEED_50M(GPIOA_SWDIO) |          \
                                     PIN_OSPEED_50M(GPIOA_SWCLK) |          \
                                     PIN_OSPEED_50M(GPIOA_NC_15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_GPIO_ADC) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LED) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_NC_2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_NC_3) |       \
                                     PIN_PUPDR_FLOATING(GPIOA_CAN_SPEED) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC_I_MOT) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_ADC_V_BAT) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOA_NC_7) |       \
                                     PIN_PUPDR_PULLUP(GPIOA_MOTOR_PWM_A) |\
                                     PIN_PUPDR_PULLUP(GPIOA_MOTOR_PWM_B) |\
                                     PIN_PUPDR_PULLUP(GPIOA_MOTOR_EN_A) | \
                                     PIN_PUPDR_PULLUP(GPIOA_MOTOR_EN_B) | \
                                     PIN_PUPDR_FLOATING(GPIOA_GPIO_I) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOA_NC_15))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_GPIO_ADC) |          \
                                     PIN_ODR_HIGH(GPIOA_LED) |              \
                                     PIN_ODR_LOW(GPIOA_NC_2) |              \
                                     PIN_ODR_LOW(GPIOA_NC_3) |              \
                                     PIN_ODR_HIGH(GPIOA_CAN_SPEED) |        \
                                     PIN_ODR_LOW(GPIOA_ADC_I_MOT) |         \
                                     PIN_ODR_LOW(GPIOA_ADC_V_BAT) |         \
                                     PIN_ODR_LOW(GPIOA_NC_7) |              \
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_A) |       \
                                     PIN_ODR_LOW(GPIOA_MOTOR_PWM_B) |       \
                                     PIN_ODR_LOW(GPIOA_MOTOR_EN_A) |        \
                                     PIN_ODR_LOW(GPIOA_MOTOR_EN_B) |        \
                                     PIN_ODR_LOW(GPIOA_GPIO_I) |            \
                                     PIN_ODR_LOW(GPIOA_SWDIO) |             \
                                     PIN_ODR_LOW(GPIOA_SWCLK) |             \
                                     PIN_ODR_LOW(GPIOA_NC_15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_GPIO_ADC, 0) |       \
                                     PIN_AFIO_AF(GPIOA_LED, 0) |            \
                                     PIN_AFIO_AF(GPIOA_NC_2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_NC_3, 0) |           \
                                     PIN_AFIO_AF(GPIOA_CAN_SPEED, 0) |      \
                                     PIN_AFIO_AF(GPIOA_ADC_I_MOT, 0) |      \
                                     PIN_AFIO_AF(GPIOA_ADC_V_BAT, 0) |      \
                                     PIN_AFIO_AF(GPIOA_NC_7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_MOTOR_PWM_A, 6) |    \
                                     PIN_AFIO_AF(GPIOA_MOTOR_PWM_B, 6) |    \
                                     PIN_AFIO_AF(GPIOA_MOTOR_EN_A, 0) |     \
                                     PIN_AFIO_AF(GPIOA_MOTOR_EN_B, 0) |     \
                                     PIN_AFIO_AF(GPIOA_GPIO_I, 0) |         \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) |          \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) |          \
                                     PIN_AFIO_AF(GPIOA_NC_15, 0))

/*
 * GPIOB setup:
 *
 * PB0  - GPIOB_NC_0                (input pulldown).
 * PB1  - GPIOB_NC_1                (input pulldown).
 * PB2  - GPIOB_NC_2                (input pulldown).
 * PB3  - GPIOB_NC_3                (input pulldown).
 * PB4  - GPIOB_GPIO_A              (input floating). // alternate 2?
 * PB5  - GPIOB_GPIO_B              (input floating). // alternate 2?
 * PB6  - GPIOB_QUAD_ENC_A          (alternate 2 (TIM4_CH1)).
 * PB7  - GPIOB_QUAD_ENC_B          (alternate 2 (TIM4_CH2)).
 * PB8  - GPIOB_CAN_RX              (alternate 9 (CAN_RX)).
 * PB9  - GPIOB_CAN_TX              (alternate 9 (CAN_TX)).
 * PB10 - GPIOB_USART3_TX           (alternate 7 (USART3_TX)).
 * PB11 - GPIOB_USART3_RX           (alternate 7 (USART3_RX)).
 * PB12 - GPIOB_NC_12               (input pulldown).
 * PB13 - GPIOB_NC_13               (input pulldown).
 * PB14 - GPIOB_NC_14               (input pulldown).
 * PB15 - GPIOB_NC_15               (input pulldown).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_NC_0) |           \
                                     PIN_MODE_INPUT(GPIOB_NC_1) |           \
                                     PIN_MODE_INPUT(GPIOB_NC_2) |           \
                                     PIN_MODE_INPUT(GPIOB_NC_3) |           \
                                     PIN_MODE_INPUT(GPIOB_GPIO_A) |         \
                                     PIN_MODE_INPUT(GPIOB_GPIO_B) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_QUAD_ENC_A) | \
                                     PIN_MODE_ALTERNATE(GPIOB_QUAD_ENC_B) | \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_RX) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_USART3_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_USART3_RX) |  \
                                     PIN_MODE_INPUT(GPIOB_NC_12) |          \
                                     PIN_MODE_INPUT(GPIOB_NC_13) |          \
                                     PIN_MODE_INPUT(GPIOB_NC_14) |          \
                                     PIN_MODE_INPUT(GPIOB_NC_15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_NC_0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIO_A) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_GPIO_B) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QUAD_ENC_A) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_QUAD_ENC_B) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_TX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_NC_15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_50M(GPIOB_NC_0) |           \
                                     PIN_OSPEED_50M(GPIOB_NC_1) |           \
                                     PIN_OSPEED_50M(GPIOB_NC_2) |           \
                                     PIN_OSPEED_50M(GPIOB_NC_3) |           \
                                     PIN_OSPEED_50M(GPIOB_GPIO_A) |         \
                                     PIN_OSPEED_50M(GPIOB_GPIO_B) |         \
                                     PIN_OSPEED_50M(GPIOB_QUAD_ENC_A) |     \
                                     PIN_OSPEED_50M(GPIOB_QUAD_ENC_B) |     \
                                     PIN_OSPEED_50M(GPIOB_CAN_RX) |         \
                                     PIN_OSPEED_50M(GPIOB_CAN_TX) |         \
                                     PIN_OSPEED_50M(GPIOB_USART3_TX) |      \
                                     PIN_OSPEED_50M(GPIOB_USART3_RX) |      \
                                     PIN_OSPEED_50M(GPIOB_NC_12) |          \
                                     PIN_OSPEED_50M(GPIOB_NC_13) |          \
                                     PIN_OSPEED_50M(GPIOB_NC_14) |          \
                                     PIN_OSPEED_50M(GPIOB_NC_15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_NC_0) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_1) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_2) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_3) |       \
                                     PIN_PUPDR_FLOATING(GPIOB_GPIO_A) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_GPIO_B) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_QUAD_ENC_A) | \
                                     PIN_PUPDR_PULLUP(GPIOB_QUAD_ENC_B) | \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_TX) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_RX) |  \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_12) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOB_NC_15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_NC_0) |              \
                                     PIN_ODR_LOW(GPIOB_NC_1) |              \
                                     PIN_ODR_LOW(GPIOB_NC_2) |              \
                                     PIN_ODR_LOW(GPIOB_NC_3) |              \
                                     PIN_ODR_LOW(GPIOB_GPIO_A) |            \
                                     PIN_ODR_LOW(GPIOB_GPIO_B) |            \
                                     PIN_ODR_LOW(GPIOB_QUAD_ENC_A) |        \
                                     PIN_ODR_LOW(GPIOB_QUAD_ENC_B) |        \
                                     PIN_ODR_LOW(GPIOB_CAN_RX) |            \
                                     PIN_ODR_LOW(GPIOB_CAN_TX) |            \
                                     PIN_ODR_LOW(GPIOB_USART3_TX) |         \
                                     PIN_ODR_LOW(GPIOB_USART3_RX) |         \
                                     PIN_ODR_LOW(GPIOB_NC_12) |             \
                                     PIN_ODR_LOW(GPIOB_NC_13) |             \
                                     PIN_ODR_LOW(GPIOB_NC_14) |             \
                                     PIN_ODR_LOW(GPIOB_NC_15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_NC_0, 0) |           \
                                     PIN_AFIO_AF(GPIOB_NC_1, 0) |           \
                                     PIN_AFIO_AF(GPIOB_NC_2, 0) |           \
                                     PIN_AFIO_AF(GPIOB_NC_3, 0) |           \
                                     PIN_AFIO_AF(GPIOB_GPIO_A, 0) |         \
                                     PIN_AFIO_AF(GPIOB_GPIO_B, 0) |         \
                                     PIN_AFIO_AF(GPIOB_QUAD_ENC_A, 2) |     \
                                     PIN_AFIO_AF(GPIOB_QUAD_ENC_B, 2))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_CAN_RX, 9) |         \
                                     PIN_AFIO_AF(GPIOB_CAN_TX, 9) |         \
                                     PIN_AFIO_AF(GPIOB_USART3_TX, 7) |      \
                                     PIN_AFIO_AF(GPIOB_USART3_RX, 7) |      \
                                     PIN_AFIO_AF(GPIOB_NC_12, 0) |          \
                                     PIN_AFIO_AF(GPIOB_NC_13, 0) |          \
                                     PIN_AFIO_AF(GPIOB_NC_14, 0) |          \
                                     PIN_AFIO_AF(GPIOB_NC_15, 0))

/*
 * GPIOC setup:
 *
 * PC13 - GPIOC_NC_13               (input pulldown).
 * PC14 - GPIOC_NC_14               (input pulldown).
 * PC15 - GPIOC_NC_15               (input pulldown).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_NC_13) |          \
                                     PIN_MODE_INPUT(GPIOC_NC_14) |          \
                                     PIN_MODE_INPUT(GPIOC_NC_15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_NC_13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_NC_14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_NC_15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_50M(GPIOC_NC_13) |          \
                                     PIN_OSPEED_50M(GPIOC_NC_14) |          \
                                     PIN_OSPEED_50M(GPIOC_NC_15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOC_NC_13) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_NC_14) |      \
                                     PIN_PUPDR_PULLDOWN(GPIOC_NC_15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_NC_13) |             \
                                     PIN_ODR_LOW(GPIOC_NC_14) |             \
                                     PIN_ODR_LOW(GPIOC_NC_15))
#define VAL_GPIOC_AFRL              (0)
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_NC_13, 0) |          \
                                     PIN_AFIO_AF(GPIOC_NC_14, 0) |          \
                                     PIN_AFIO_AF(GPIOC_NC_15, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (0)
#define VAL_GPIOD_OTYPER            (0)
#define VAL_GPIOD_OSPEEDR           (0)
#define VAL_GPIOD_PUPDR             (0)
#define VAL_GPIOD_ODR               (0)
#define VAL_GPIOD_AFRL              (0)
#define VAL_GPIOD_AFRH              (0)

/*
 * GPIOE setup:
 */
#define VAL_GPIOE_MODER             (0)
#define VAL_GPIOE_OTYPER            (0)
#define VAL_GPIOE_OSPEEDR           (0)
#define VAL_GPIOE_PUPDR             (0)
#define VAL_GPIOE_ODR               (0)
#define VAL_GPIOE_AFRL              (0)
#define VAL_GPIOE_AFRH              (0)

/*
 * GPIOF setup:
 *
 * PF0  - OSC_IN                    (input floating).
 * PF1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_OSC_OUT))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_50M(GPIOF_OSC_IN) |        \
                                     PIN_OSPEED_50M(GPIOF_OSC_OUT))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_OSC_IN) |           \
                                     PIN_ODR_LOW(GPIOF_OSC_OUT))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0))
#define VAL_GPIOF_AFRH              (0)


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
    void boardInit(void);
    void can_transceiver_activate(void);
    void can_transceiver_standby(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
/* clang-format on */
