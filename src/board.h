#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Board identifier.
 */
#define BOARD_CVRA_UWB_BEACON
#define BOARD_NAME                  "CVRA UWB Beacon"


/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#define STM32_LSECLK                0U
#define STM32_HSECLK                16000000U

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F405xx

/*
 *  Define for functional usb without VBUS detection
 */
#define BOARD_OTG_NOVBUSSENS 1

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0U
#define GPIOA_PIN1                  1U
#define GPIOA_UART2_TX              2U
#define GPIOA_UART2_RX              3U
#define GPIOA_PIN4                  4U
#define GPIOA_PIN5                  5U
#define GPIOA_PIN6                  6U
#define GPIOA_PIN7                  7U
#define GPIOA_PIN8                  8U
#define GPIOA_PIN9                  9U
#define GPIOA_PIN10                 10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_LED_ERROR             0U
#define GPIOB_LED_DEBUG             1U
#define GPIOB_LED_STATUS            2U
#define GPIOB_SWO                   3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_PIN10                 10U
#define GPIOB_IMU_INT               11U
#define GPIOB_IMU_CS_N              12U
#define GPIOB_IMU_SCK               13U
#define GPIOB_IMU_MISO              14U
#define GPIOB_IMU_MOSI              15U

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

#define GPIOD_PIN2                  0U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U

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
 * GPIOA_PIN0                       (input floating).
 * GPIOA_PIN1                       (input floating).
 * GPIOA_UART2_TX                   (alternate 7).
 * GPIOA_UART2_RX                   (alternate 7).
 * GPIOA_PIN4                       (input floating).
 * GPIOA_PIN5                       (input floating).
 * GPIOA_PIN6                       (input floating).
 * GPIOA_PIN7                       (input floating).
 * GPIOA_PIN8                       (input floating).
 * GPIOA_PIN9                       (input floating).
 * GPIOA_PIN10                      (input floating).
 * GPIOA_OTG_FS_DM                  (alternate 10).
 * GPIOA_OTG_FS_DP                  (alternate 10).
 * GPIOA_SWDIO                      (alternate 0).
 * GPIOA_SWCLK                      (alternate 0).
 * GPIOA_PIN15                      (input pullup).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) | \
                                     PIN_MODE_INPUT(GPIOA_PIN1) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_UART2_RX) | \
                                     PIN_MODE_INPUT(GPIOA_PIN4) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_INPUT(GPIOA_PIN6) | \
                                     PIN_MODE_INPUT(GPIOA_PIN7) | \
                                     PIN_MODE_INPUT(GPIOA_PIN8) | \
                                     PIN_MODE_INPUT(GPIOA_PIN9) | \
                                     PIN_MODE_INPUT(GPIOA_PIN10) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) | \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART2_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_PIN0) | \
                                     PIN_OSPEED_100M(GPIOA_PIN1) | \
                                     PIN_OSPEED_100M(GPIOA_UART2_TX) | \
                                     PIN_OSPEED_100M(GPIOA_UART2_RX) | \
                                     PIN_OSPEED_100M(GPIOA_PIN4) | \
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PIN6) | \
                                     PIN_OSPEED_100M(GPIOA_PIN7) | \
                                     PIN_OSPEED_100M(GPIOA_PIN8) | \
                                     PIN_OSPEED_100M(GPIOA_PIN9) | \
                                     PIN_OSPEED_100M(GPIOA_PIN10) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK) | \
                                     PIN_OSPEED_100M(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOA_UART2_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOA_UART2_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) | \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) | \
                                     PIN_ODR_HIGH(GPIOA_PIN1) | \
                                     PIN_ODR_HIGH(GPIOA_UART2_TX) | \
                                     PIN_ODR_HIGH(GPIOA_UART2_RX) | \
                                     PIN_ODR_HIGH(GPIOA_PIN4) | \
                                     PIN_ODR_HIGH(GPIOA_PIN5) | \
                                     PIN_ODR_HIGH(GPIOA_PIN6) | \
                                     PIN_ODR_HIGH(GPIOA_PIN7) | \
                                     PIN_ODR_HIGH(GPIOA_PIN8) | \
                                     PIN_ODR_HIGH(GPIOA_PIN9) | \
                                     PIN_ODR_HIGH(GPIOA_PIN10) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) | \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) | \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOA_UART2_TX, 7) | \
                                     PIN_AFIO_AF(GPIOA_UART2_RX, 7) | \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN7, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN15, 0))

/*
 * GPIOB setup:
 *
 * GPIOB_LED_ERROR                  (output).
 * GPIOB_LED_DEBUG                  (output).
 * GPIOB_LED_STATUS                 (output).
 * GPIOB_SWO                        (alternate 0).
 * GPIOB_PIN4                       (input floating).
 * GPIOB_PIN5                       (input floating).
 * GPIOB_PIN6                       (input floating).
 * GPIOB_PIN7                       (input floating).
 * GPIOB_PIN8                       (input floating).
 * GPIOB_PIN9                       (input floating).
 * GPIOB_PIN10                      (input floating).
 * GPIOB_IMU_INT                    (input floating).
 * GPIOB_IMU_CS_N                   (output high).
 * GPIOB_IMU_SCK                    (alternate 5).
 * GPIOB_IMU_MISO                   (alternate 5).
 * GPIOB_IMU_MOSI                   (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_LED_ERROR) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_DEBUG) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_STATUS) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO) | \
                                     PIN_MODE_INPUT(GPIOB_PIN4) | \
                                     PIN_MODE_INPUT(GPIOB_PIN5) | \
                                     PIN_MODE_INPUT(GPIOB_PIN6) | \
                                     PIN_MODE_INPUT(GPIOB_PIN7) | \
                                     PIN_MODE_INPUT(GPIOB_PIN8) | \
                                     PIN_MODE_INPUT(GPIOB_PIN9) | \
                                     PIN_MODE_INPUT(GPIOB_PIN10) | \
                                     PIN_MODE_INPUT(GPIOB_IMU_INT) | \
                                     PIN_MODE_OUTPUT(GPIOB_IMU_CS_N) | \
                                     PIN_MODE_ALTERNATE(GPIOB_IMU_SCK) | \
                                     PIN_MODE_ALTERNATE(GPIOB_IMU_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_IMU_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LED_ERROR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_DEBUG) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_STATUS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_INT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_CS_N) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_SCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_LED_ERROR) | \
                                     PIN_OSPEED_100M(GPIOB_LED_DEBUG) | \
                                     PIN_OSPEED_100M(GPIOB_LED_STATUS) | \
                                     PIN_OSPEED_100M(GPIOB_SWO) | \
                                     PIN_OSPEED_100M(GPIOB_PIN4) | \
                                     PIN_OSPEED_100M(GPIOB_PIN5) | \
                                     PIN_OSPEED_100M(GPIOB_PIN6) | \
                                     PIN_OSPEED_100M(GPIOB_PIN7) | \
                                     PIN_OSPEED_100M(GPIOB_PIN8) | \
                                     PIN_OSPEED_100M(GPIOB_PIN9) | \
                                     PIN_OSPEED_100M(GPIOB_PIN10) | \
                                     PIN_OSPEED_100M(GPIOB_IMU_INT) | \
                                     PIN_OSPEED_100M(GPIOB_IMU_CS_N) | \
                                     PIN_OSPEED_100M(GPIOB_IMU_SCK) | \
                                     PIN_OSPEED_100M(GPIOB_IMU_MISO) | \
                                     PIN_OSPEED_100M(GPIOB_IMU_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LED_ERROR) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_DEBUG) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_STATUS) | \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_INT) | \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_CS_N) | \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_SCK) | \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_MISO) | \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_LED_ERROR) | \
                                     PIN_ODR_LOW(GPIOB_LED_DEBUG) | \
                                     PIN_ODR_LOW(GPIOB_LED_STATUS) | \
                                     PIN_ODR_HIGH(GPIOB_SWO) | \
                                     PIN_ODR_HIGH(GPIOB_PIN4) | \
                                     PIN_ODR_HIGH(GPIOB_PIN5) | \
                                     PIN_ODR_HIGH(GPIOB_PIN6) | \
                                     PIN_ODR_HIGH(GPIOB_PIN7) | \
                                     PIN_ODR_HIGH(GPIOB_PIN8) | \
                                     PIN_ODR_HIGH(GPIOB_PIN9) | \
                                     PIN_ODR_HIGH(GPIOB_PIN10) | \
                                     PIN_ODR_HIGH(GPIOB_IMU_INT) | \
                                     PIN_ODR_HIGH(GPIOB_IMU_CS_N) | \
                                     PIN_ODR_HIGH(GPIOB_IMU_SCK) | \
                                     PIN_ODR_HIGH(GPIOB_IMU_MISO) | \
                                     PIN_ODR_HIGH(GPIOB_IMU_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LED_ERROR, 0) | \
                                     PIN_AFIO_AF(GPIOB_LED_DEBUG, 0) | \
                                     PIN_AFIO_AF(GPIOB_LED_STATUS, 0) | \
                                     PIN_AFIO_AF(GPIOB_SWO, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOB_IMU_INT, 0) | \
                                     PIN_AFIO_AF(GPIOB_IMU_CS_N, 0) | \
                                     PIN_AFIO_AF(GPIOB_IMU_SCK, 5) | \
                                     PIN_AFIO_AF(GPIOB_IMU_MISO, 5) | \
                                     PIN_AFIO_AF(GPIOB_IMU_MOSI, 5))

/*
 * GPIOC setup:
 *
 * GPIOC_PIN0                       (input floating).
 * GPIOC_PIN1                       (input floating).
 * GPIOC_PIN2                       (input floating).
 * GPIOC_PIN3                       (input floating).
 * GPIOC_PIN4                       (input floating).
 * GPIOC_PIN5                       (input floating).
 * GPIOC_PIN6                       (input floating).
 * GPIOC_PIN7                       (input floating).
 * GPIOC_PIN8                       (input floating).
 * GPIOC_PIN9                       (input floating).
 * GPIOC_PIN10                      (input floating).
 * GPIOC_PIN11                      (input floating).
 * GPIOC_PIN12                      (input floating).
 * GPIOC_PIN13                      (input floating).
 * GPIOC_PIN14                      (input floating).
 * GPIOC_PIN15                      (input floating).
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
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_PIN0) | \
                                     PIN_ODR_HIGH(GPIOC_PIN1) | \
                                     PIN_ODR_HIGH(GPIOC_PIN2) | \
                                     PIN_ODR_HIGH(GPIOC_PIN3) | \
                                     PIN_ODR_HIGH(GPIOC_PIN4) | \
                                     PIN_ODR_HIGH(GPIOC_PIN5) | \
                                     PIN_ODR_HIGH(GPIOC_PIN6) | \
                                     PIN_ODR_HIGH(GPIOC_PIN7) | \
                                     PIN_ODR_HIGH(GPIOC_PIN8) | \
                                     PIN_ODR_HIGH(GPIOC_PIN9) | \
                                     PIN_ODR_HIGH(GPIOC_PIN10) | \
                                     PIN_ODR_HIGH(GPIOC_PIN11) | \
                                     PIN_ODR_HIGH(GPIOC_PIN12) | \
                                     PIN_ODR_HIGH(GPIOC_PIN13) | \
                                     PIN_ODR_HIGH(GPIOC_PIN14) | \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
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
 * GPIOD_PIN2                       (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN2))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN2))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_PIN2))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN2))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN2))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN2, 0))
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
 */
#define VAL_GPIOF_MODER             (0)
#define VAL_GPIOF_OTYPER            (0)
#define VAL_GPIOF_OSPEEDR           (0)
#define VAL_GPIOF_PUPDR             (0)
#define VAL_GPIOF_ODR               (0)
#define VAL_GPIOF_AFRL              (0)
#define VAL_GPIOF_AFRH              (0)

/*
 * GPIOG setup:
 */
#define VAL_GPIOG_MODER             (0)
#define VAL_GPIOG_OTYPER            (0)
#define VAL_GPIOG_OSPEEDR           (0)
#define VAL_GPIOG_PUPDR             (0)
#define VAL_GPIOG_ODR               (0)
#define VAL_GPIOG_AFRL              (0)
#define VAL_GPIOG_AFRH              (0)

/*
 * GPIOH setup:
 *
 * GPIOH_OSC_IN                     (input floating).
 * GPIOH_OSC_OUT                    (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) | \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_100M(GPIOH_OSC_IN) | \
                                     PIN_OSPEED_100M(GPIOH_OSC_OUT))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) | \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) | \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0) | \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0))
#define VAL_GPIOH_AFRH              (0)

/*
 * GPIOI setup:
 */
#define VAL_GPIOI_MODER             (0)
#define VAL_GPIOI_OTYPER            (0)
#define VAL_GPIOI_OSPEEDR           (0)
#define VAL_GPIOI_PUPDR             (0)
#define VAL_GPIOI_ODR               (0)
#define VAL_GPIOI_AFRL              (0)
#define VAL_GPIOI_AFRH              (0)


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
