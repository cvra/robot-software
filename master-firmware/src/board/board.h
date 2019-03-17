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
#define BOARD_ST_NUCLEO144_F429ZI
#define BOARD_NAME                  "STM32 Nucleo144-F429ZI"

/*
 * Ethernet PHY type.
 */
#ifndef MII_LAN8742A_ID
#define MII_LAN8742A_ID 0x0007C130
#endif
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768
#endif

/* on board 10MHz crystal */
#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330

/*
 * MCU type as defined in the ST header.
 */
#define STM32F429xx

/*
 * IO pins assignments.
 */
#define GPIOA_PIN0                  0
#define GPIOA_ETH_RMII_REF_CLK      1 // RMII_REF_CLK (alternate 11)
#define GPIOA_ETH_RMII_MDIO         2 // RMII_MDIO (alternate 11)
#define GPIOA_PIN3                  3
#define GPIOA_PIN4                  4
#define GPIOA_SD_DETECT             5 // SD_DETECT (input)
#define GPIOA_SD_PWR                6 // SD_PWR (output high)
#define GPIOA_ETH_RMII_CRS_DV       7 // RMII RX data valid (alternate 11)
#define GPIOA_PIN8                  8 // (USB_SOF) (input floating)
#define GPIOA_USB_VBUS              9 // USB_VBUS (input pulldown)
#define GPIOA_OTG_FS_ID             10 // USB_ID (alternate 10)
#define GPIOA_OTG_FS_DM             11 // USB_DM (alternate 10)
#define GPIOA_OTG_FS_DP             12 // USB_DP (alternate 10)
#define GPIOA_SWDIO_JTAG_TMS        13 // SWDIO (alternate 0)
#define GPIOA_SWCLK_JTAG_TCK        14 // SWCLK (alternate 0)
#define GPIOA_JTAG_TDI              15 // JTAG_TDI (alternate 0)

#define GPIOB_LED_GREEN             0 // LED_GREEN (output pushpull)
#define GPIOB_PIN1                  1
#define GPIOB_BOOT1                 2 // BOOT1 (input floating)
#define GPIOB_SWO_JTAG_TDO          3 // SWO (alternate 0)
#define GPIOB_JTAG_TRST             4 // (alternate 0)
#define GPIOB_CAN2_RX               5 // CAN2_RX (alternate 9)
#define GPIOB_CAN2_TX               6 // CAN2_TX (alternate 9)
#define GPIOB_LED_BLUE              7 // LED_BLUE (output pushpull)
#define GPIOB_I2C1_SCL              8 // I2C1_SCL (alternate 4)
#define GPIOB_I2C1_SDA              9 // I2C1_SDA (alternate 4)
#define GPIOB_I2C2_SCL              10 // I2C2_SCL (alternate 4)
#define GPIOB_I2C2_SDA              11 // I2C2_SDA (alternate 4)
#define GPIOB_LED_SD                12 // LED_SD (output pushpull)
#define GPIOB_ETH_RMII_TXD1         13 // RMII_TXD1 (alternate 11)
#define GPIOB_LED_RED               14 // LED_RED (output pushpull)
#define GPIOB_ENCODER_RIGHT_CHI     15 // ENCODER_RIGHT_CHI (input pullup)

#define GPIOC_SERVO_OUTPUT_EN       0 // SERVO_OUTPUT_EN (output high)
#define GPIOC_ETH_RMII_MDC          1 // RMII_MDC (alternate 11)
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_ETH_RMII_RXD0         4 // RMII_RXD0 (alternate 11)
#define GPIOC_ETH_RMII_RXD1         5 // RMII_RXD1 (alternate 11)
#define GPIOC_ENCODER_RIGHT_CHA     6 // TIM3_CH1 (alternate 2)
#define GPIOC_ENCODER_RIGHT_CHB     7 // TIM3_CH2 (alternate 2)
#define GPIOC_SD_D0                 8 // SDC_D0 (alternate 12)
#define GPIOC_SD_D1                 9 // SDC_D1 (alternate 12)
#define GPIOC_SD_D2                 10 // SDC_D2 (alternate 12)
#define GPIOC_SD_D3                 11 // SDC_D3 (alternate 12)
#define GPIOC_SD_CLK                12 // SDC_CLK (alternate 12)
#define GPIOC_USER_BUTTON           13 // USER_BUTTON (input floating)
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_CAN1_RX               0 // CAN1_RX (alternate 9)
#define GPIOD_CAN1_TX               1 // CAN1_TX (alternate 9)
#define GPIOD_SD_CMD                2 // SD_CMD (alternate 12)
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_UART2_TX              5 // USART2_TX (alternate 7)
#define GPIOD_UART2_RX              6 // USART2_RX (alternate 7, pullup)
#define GPIOD_PIN7                  7
#define GPIOD_PIN8                  8
#define GPIOD_EXT_IO_2              9 // EXT_IO_2
#define GPIOD_LED_RGB_R             10 // LED_RGB_R
#define GPIOD_ENCODER_LEFT_CHI      11 // (input floating)
#define GPIOD_ENCODER_LEFT_CHA      12 // TIM4_CH1 (alternate 2)
#define GPIOD_ENCODER_LEFT_CHB      13 // TIM4_CH2 (alternate 2)
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_EXT_IO_8              0 // EXT_IO_8
#define GPIOE_EXT_IO_0              1 // EXT_IO_0
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_ENCODER_TRANSVERS_CHI 4 // (input floating)
#define GPIOE_ENCODER_TRANSVERS_CHA 5 // TIM9_CH1 (alternate 3)
#define GPIOE_ENCODER_TRANSVERS_CHB 6 // TIM9_CH2 (alternate 3)
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_SPI4_CS               11 // SPI4_CS (output pushpull)
#define GPIOE_SPI4_SCK              12 // SPI4_SCK (alternate 5)
#define GPIOE_LCD_RSTN             13  // LCD screen reset (alternate 5)
#define GPIOE_SPI4_MOSI             14 // SPI4_MOSI (alternate 5)
#define GPIOE_PIN15                 15

#define GPIOF_PIN0                  0
#define GPIOF_PIN1                  1
#define GPIOF_PIN2                  2
#define GPIOF_ROBOT_SELECT          3 // ROBOT_SELECT (input up), default = high
#define GPIOF_PIN4                  4
#define GPIOF_PIN5                  5
#define GPIOF_UART7_RX              6 // DEBUG_UART_RX (alternate 8, pullup)
#define GPIOF_UART7_TX              7 // DEBUG_UART_TX (alternate 8)
#define GPIOF_PIN8                  8
#define GPIOF_EXT_IO_5              9 // EXT_IO_5
#define GPIOF_PIN10                 10
#define GPIOF_EXT_IO_9              11 // EXT_IO_9
#define GPIOF_EXT_IO_7              12 // EXT_IO_7
#define GPIOF_PIN13                 13
#define GPIOF_PIN14                 14
#define GPIOF_EXT_IO_6              15 // EXT_IO_6

#define GPIOG_EXT_IO_1              0 // EXT_IO_1
#define GPIOG_EXT_IO_4              1 // EXT_IO_4
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_LED_RGB_G             4 // LED_RGB_G
#define GPIOG_LED_RGB_B             5 // LED_RGB_B
#define GPIOG_USB_OTG_POWER_EN      6 // USB device 5V VBUS power switch (output low)
#define GPIOG_USB_OVERCURRENT       7 // USB_OverCurrent, active low (for USB OTG host) (input floating)
#define GPIOG_EXT_IO_10             8 // EXT_IO_10
#define GPIOG_PIN9                  9 // GPIO1 (USART6_RX)
#define GPIOG_EXT_IO_3              10 // EXT_IO_3
#define GPIOG_ETH_RMII_TXEN         11 // RMII_TX_EN (alternate 11)
#define GPIOG_PIN12                 12 // GPIO4
#define GPIOG_ETH_RMII_TXD0         13 // RMII_TXD0 (alternate 11)
#define GPIOG_PIN14                 14 // GPIO2 (USART6_TX)
#define GPIOG_PIN15                 15 // GPIO3

#define GPIOH_OSC_IN                0 // input floating
#define GPIOH_OSC_OUT               1 // input floating

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
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * GPIOA setup:
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) | \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_MODE_INPUT(GPIOA_PIN3) | \
                                     PIN_MODE_INPUT(GPIOA_PIN4) | \
                                     PIN_MODE_INPUT(GPIOA_SD_DETECT) | \
                                     PIN_MODE_OUTPUT(GPIOA_SD_PWR) | \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_CRS_DV) | \
                                     PIN_MODE_INPUT(GPIOA_PIN8) | \
                                     PIN_MODE_INPUT(GPIOA_USB_VBUS) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_ID) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO_JTAG_TMS) | \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK_JTAG_TCK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SD_DETECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SD_PWR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_ETH_RMII_CRS_DV) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_VBUS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_ID) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO_JTAG_TMS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK_JTAG_TCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_100M(GPIOA_PIN0) | \
                                     PIN_OSPEED_100M(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_OSPEED_100M(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_OSPEED_100M(GPIOA_PIN3) | \
                                     PIN_OSPEED_100M(GPIOA_PIN4) | \
                                     PIN_OSPEED_100M(GPIOA_SD_DETECT) | \
                                     PIN_OSPEED_100M(GPIOA_SD_PWR) | \
                                     PIN_OSPEED_100M(GPIOA_ETH_RMII_CRS_DV) | \
                                     PIN_OSPEED_100M(GPIOA_PIN8) | \
                                     PIN_OSPEED_100M(GPIOA_USB_VBUS) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_ID) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DM) | \
                                     PIN_OSPEED_100M(GPIOA_OTG_FS_DP) | \
                                     PIN_OSPEED_100M(GPIOA_SWDIO_JTAG_TMS) | \
                                     PIN_OSPEED_100M(GPIOA_SWCLK_JTAG_TCK) | \
                                     PIN_OSPEED_100M(GPIOA_JTAG_TDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) | \
                                     PIN_PUPDR_PULLUP(GPIOA_SD_DETECT) | \
                                     PIN_PUPDR_FLOATING(GPIOA_SD_PWR) | \
                                     PIN_PUPDR_FLOATING(GPIOA_ETH_RMII_CRS_DV) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_USB_VBUS) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_ID) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) | \
                                     PIN_PUPDR_PULLUP(GPIOA_SWDIO_JTAG_TMS) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK_JTAG_TCK) | \
                                     PIN_PUPDR_PULLUP(GPIOA_JTAG_TDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_PIN0) | \
                                     PIN_ODR_HIGH(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_ODR_HIGH(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_ODR_HIGH(GPIOA_PIN3) | \
                                     PIN_ODR_HIGH(GPIOA_PIN4) | \
                                     PIN_ODR_HIGH(GPIOA_SD_DETECT) | \
                                     PIN_ODR_HIGH(GPIOA_SD_PWR) | \
                                     PIN_ODR_HIGH(GPIOA_ETH_RMII_CRS_DV) | \
                                     PIN_ODR_HIGH(GPIOA_PIN8) | \
                                     PIN_ODR_HIGH(GPIOA_USB_VBUS) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_ID) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) | \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) | \
                                     PIN_ODR_HIGH(GPIOA_SWDIO_JTAG_TMS) | \
                                     PIN_ODR_HIGH(GPIOA_SWCLK_JTAG_TCK) | \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOA_ETH_RMII_REF_CLK, 11) | \
                                     PIN_AFIO_AF(GPIOA_ETH_RMII_MDIO, 11) | \
                                     PIN_AFIO_AF(GPIOA_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOA_SD_DETECT, 0) | \
                                     PIN_AFIO_AF(GPIOA_SD_PWR, 0) | \
                                     PIN_AFIO_AF(GPIOA_ETH_RMII_CRS_DV, 11))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOA_USB_VBUS, 0) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_ID, 10) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10) | \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10) | \
                                     PIN_AFIO_AF(GPIOA_SWDIO_JTAG_TMS, 0) | \
                                     PIN_AFIO_AF(GPIOA_SWCLK_JTAG_TCK, 0) | \
                                     PIN_AFIO_AF(GPIOA_JTAG_TDI, 0))

/*
 * GPIOB setup:
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_LED_GREEN) | \
                                     PIN_MODE_INPUT(GPIOB_PIN1) | \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TRST) | \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN2_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOB_CAN2_TX) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_BLUE) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SCL) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C2_SDA) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_SD) | \
                                     PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_RED) | \
                                     PIN_MODE_INPUT(GPIOB_ENCODER_RIGHT_CHI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LED_GREEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TRST) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_CAN2_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_BLUE) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SCL) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C2_SDA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_SD) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_RED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ENCODER_RIGHT_CHI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_LED_GREEN) | \
                                     PIN_OSPEED_100M(GPIOB_PIN1) | \
                                     PIN_OSPEED_100M(GPIOB_BOOT1) | \
                                     PIN_OSPEED_100M(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_OSPEED_100M(GPIOB_JTAG_TRST) | \
                                     PIN_OSPEED_100M(GPIOB_CAN2_RX) | \
                                     PIN_OSPEED_100M(GPIOB_CAN2_TX) | \
                                     PIN_OSPEED_100M(GPIOB_LED_BLUE) | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SCL) | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SDA) | \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SCL) | \
                                     PIN_OSPEED_100M(GPIOB_I2C2_SDA) | \
                                     PIN_OSPEED_100M(GPIOB_LED_SD) | \
                                     PIN_OSPEED_100M(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_OSPEED_100M(GPIOB_LED_RED) | \
                                     PIN_OSPEED_100M(GPIOB_ENCODER_RIGHT_CHI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LED_GREEN) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_PUPDR_PULLUP(GPIOB_JTAG_TRST) | \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN2_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOB_CAN2_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_BLUE) | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SCL) | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C2_SDA) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_SD) | \
                                     PIN_PUPDR_FLOATING(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_RED) | \
                                     PIN_PUPDR_PULLUP(GPIOB_ENCODER_RIGHT_CHI))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_LED_GREEN) | \
                                     PIN_ODR_LOW(GPIOB_PIN1) | \
                                     PIN_ODR_LOW(GPIOB_BOOT1) | \
                                     PIN_ODR_LOW(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_ODR_LOW(GPIOB_JTAG_TRST) | \
                                     PIN_ODR_LOW(GPIOB_CAN2_RX) | \
                                     PIN_ODR_LOW(GPIOB_CAN2_TX) | \
                                     PIN_ODR_LOW(GPIOB_LED_BLUE) | \
                                     PIN_ODR_LOW(GPIOB_I2C1_SCL) | \
                                     PIN_ODR_LOW(GPIOB_I2C1_SDA) | \
                                     PIN_ODR_LOW(GPIOB_I2C2_SCL) | \
                                     PIN_ODR_LOW(GPIOB_I2C2_SDA) | \
                                     PIN_ODR_LOW(GPIOB_LED_SD) | \
                                     PIN_ODR_LOW(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_ODR_LOW(GPIOB_LED_RED) | \
                                     PIN_ODR_LOW(GPIOB_ENCODER_RIGHT_CHI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LED_GREEN, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) | \
                                     PIN_AFIO_AF(GPIOB_SWO_JTAG_TDO, 0) | \
                                     PIN_AFIO_AF(GPIOB_JTAG_TRST, 0) | \
                                     PIN_AFIO_AF(GPIOB_CAN2_RX, 9) | \
                                     PIN_AFIO_AF(GPIOB_CAN2_TX, 9) | \
                                     PIN_AFIO_AF(GPIOB_LED_BLUE, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 4) | \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4) | \
                                     PIN_AFIO_AF(GPIOB_I2C2_SCL, 4) | \
                                     PIN_AFIO_AF(GPIOB_I2C2_SDA, 4) | \
                                     PIN_AFIO_AF(GPIOB_LED_SD, 0) | \
                                     PIN_AFIO_AF(GPIOB_ETH_RMII_TXD1, 11) | \
                                     PIN_AFIO_AF(GPIOB_LED_RED, 0) | \
                                     PIN_AFIO_AF(GPIOB_ENCODER_RIGHT_CHI, 0))

/*
 * GPIOC setup:
 */
#define VAL_GPIOC_MODER             (PIN_MODE_OUTPUT(GPIOC_SERVO_OUTPUT_EN) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_MDC) | \
                                     PIN_MODE_INPUT(GPIOC_PIN2) | \
                                     PIN_MODE_INPUT(GPIOC_PIN3) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D0) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D1) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D2) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_D3) | \
                                     PIN_MODE_ALTERNATE(GPIOC_SD_CLK) | \
                                     PIN_MODE_INPUT(GPIOC_USER_BUTTON) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_SERVO_OUTPUT_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_MDC) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_D3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SD_CLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USER_BUTTON) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_SERVO_OUTPUT_EN) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_MDC) | \
                                     PIN_OSPEED_100M(GPIOC_PIN2) | \
                                     PIN_OSPEED_100M(GPIOC_PIN3) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_OSPEED_100M(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_OSPEED_100M(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_OSPEED_100M(GPIOC_SD_D0) | \
                                     PIN_OSPEED_100M(GPIOC_SD_D1) | \
                                     PIN_OSPEED_100M(GPIOC_SD_D2) | \
                                     PIN_OSPEED_100M(GPIOC_SD_D3) | \
                                     PIN_OSPEED_100M(GPIOC_SD_CLK) | \
                                     PIN_OSPEED_100M(GPIOC_USER_BUTTON) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_SERVO_OUTPUT_EN) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_MDC) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_PUPDR_PULLUP(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_PUPDR_PULLUP(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_PUPDR_PULLUP(GPIOC_SD_D0) | \
                                     PIN_PUPDR_PULLUP(GPIOC_SD_D1) | \
                                     PIN_PUPDR_PULLUP(GPIOC_SD_D2) | \
                                     PIN_PUPDR_PULLUP(GPIOC_SD_D3) | \
                                     PIN_PUPDR_FLOATING(GPIOC_SD_CLK) | \
                                     PIN_PUPDR_FLOATING(GPIOC_USER_BUTTON) | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_SERVO_OUTPUT_EN) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_MDC) | \
                                     PIN_ODR_LOW(GPIOC_PIN2) | \
                                     PIN_ODR_LOW(GPIOC_PIN3) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_ODR_LOW(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_ODR_LOW(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_ODR_LOW(GPIOC_SD_D0) | \
                                     PIN_ODR_LOW(GPIOC_SD_D1) | \
                                     PIN_ODR_LOW(GPIOC_SD_D2) | \
                                     PIN_ODR_LOW(GPIOC_SD_D3) | \
                                     PIN_ODR_LOW(GPIOC_SD_CLK) | \
                                     PIN_ODR_LOW(GPIOC_USER_BUTTON) | \
                                     PIN_ODR_LOW(GPIOC_OSC32_IN) | \
                                     PIN_ODR_LOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_SERVO_OUTPUT_EN, 0) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_MDC, 11) | \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_RXD0, 11) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_RXD1, 11) | \
                                     PIN_AFIO_AF(GPIOC_ENCODER_RIGHT_CHA, 2) | \
                                     PIN_AFIO_AF(GPIOC_ENCODER_RIGHT_CHB, 2))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SD_D0, 12) | \
                                     PIN_AFIO_AF(GPIOC_SD_D1, 12) | \
                                     PIN_AFIO_AF(GPIOC_SD_D2, 12) | \
                                     PIN_AFIO_AF(GPIOC_SD_D3, 12) | \
                                     PIN_AFIO_AF(GPIOC_SD_CLK, 12) | \
                                     PIN_AFIO_AF(GPIOC_USER_BUTTON, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOD_SD_CMD) | \
                                     PIN_MODE_INPUT(GPIOD_PIN3) | \
                                     PIN_MODE_INPUT(GPIOD_PIN4) | \
                                     PIN_MODE_ALTERNATE(GPIOD_UART2_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOD_UART2_RX) | \
                                     PIN_MODE_INPUT(GPIOD_PIN7) | \
                                     PIN_MODE_ALTERNATE(GPIOD_PIN8) | \
                                     PIN_MODE_ALTERNATE(GPIOD_EXT_IO_2) | \
                                     PIN_MODE_OUTPUT(GPIOD_LED_RGB_R) | \
                                     PIN_MODE_INPUT(GPIOD_ENCODER_LEFT_CHI) | \
                                     PIN_MODE_ALTERNATE(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_MODE_ALTERNATE(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_MODE_INPUT(GPIOD_PIN14) | \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SD_CMD) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART2_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART2_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_EXT_IO_2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_LED_RGB_R) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENCODER_LEFT_CHI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_CAN1_RX) | \
                                     PIN_OSPEED_100M(GPIOD_CAN1_TX) | \
                                     PIN_OSPEED_100M(GPIOD_SD_CMD) | \
                                     PIN_OSPEED_100M(GPIOD_PIN3) | \
                                     PIN_OSPEED_100M(GPIOD_PIN4) | \
                                     PIN_OSPEED_100M(GPIOD_UART2_TX) | \
                                     PIN_OSPEED_100M(GPIOD_UART2_RX) | \
                                     PIN_OSPEED_100M(GPIOD_PIN7) | \
                                     PIN_OSPEED_100M(GPIOD_PIN8) | \
                                     PIN_OSPEED_100M(GPIOD_EXT_IO_2) | \
                                     PIN_OSPEED_100M(GPIOD_LED_RGB_R) | \
                                     PIN_OSPEED_100M(GPIOD_ENCODER_LEFT_CHI) | \
                                     PIN_OSPEED_100M(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_OSPEED_100M(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_OSPEED_100M(GPIOD_PIN14) | \
                                     PIN_OSPEED_100M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_CAN1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOD_CAN1_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOD_SD_CMD) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOD_UART2_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOD_UART2_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) | \
                                     PIN_PUPDR_PULLUP(GPIOD_EXT_IO_2) | \
                                     PIN_PUPDR_FLOATING(GPIOD_LED_RGB_R) | \
                                     PIN_PUPDR_PULLUP(GPIOD_ENCODER_LEFT_CHI) | \
                                     PIN_PUPDR_PULLUP(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_PUPDR_PULLUP(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_CAN1_RX) | \
                                     PIN_ODR_LOW(GPIOD_CAN1_TX) | \
                                     PIN_ODR_LOW(GPIOD_SD_CMD) | \
                                     PIN_ODR_LOW(GPIOD_PIN3) | \
                                     PIN_ODR_LOW(GPIOD_PIN4) | \
                                     PIN_ODR_LOW(GPIOD_UART2_TX) | \
                                     PIN_ODR_LOW(GPIOD_UART2_RX) | \
                                     PIN_ODR_LOW(GPIOD_PIN7) | \
                                     PIN_ODR_LOW(GPIOD_PIN8) | \
                                     PIN_ODR_LOW(GPIOD_EXT_IO_2) | \
                                     PIN_ODR_LOW(GPIOD_LED_RGB_R) | \
                                     PIN_ODR_LOW(GPIOD_ENCODER_LEFT_CHI) | \
                                     PIN_ODR_LOW(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_ODR_LOW(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_ODR_LOW(GPIOD_PIN14) | \
                                     PIN_ODR_LOW(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN1_RX, 9) | \
                                     PIN_AFIO_AF(GPIOD_CAN1_TX, 9) | \
                                     PIN_AFIO_AF(GPIOD_SD_CMD, 12) | \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOD_UART2_TX, 7) | \
                                     PIN_AFIO_AF(GPIOD_UART2_RX, 7) | \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOD_EXT_IO_2, 0) | \
                                     PIN_AFIO_AF(GPIOD_LED_RGB_R, 0) | \
                                     PIN_AFIO_AF(GPIOD_ENCODER_LEFT_CHI, 0) | \
                                     PIN_AFIO_AF(GPIOD_ENCODER_LEFT_CHA, 2) | \
                                     PIN_AFIO_AF(GPIOD_ENCODER_LEFT_CHB, 2) | \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_EXT_IO_8) | \
                                     PIN_MODE_INPUT(GPIOE_EXT_IO_0) | \
                                     PIN_MODE_INPUT(GPIOE_PIN2) | \
                                     PIN_MODE_INPUT(GPIOE_PIN3) | \
                                     PIN_MODE_INPUT(GPIOE_ENCODER_TRANSVERS_CHI) | \
                                     PIN_MODE_ALTERNATE(GPIOE_ENCODER_TRANSVERS_CHA) | \
                                     PIN_MODE_ALTERNATE(GPIOE_ENCODER_TRANSVERS_CHB) | \
                                     PIN_MODE_INPUT(GPIOE_PIN7) | \
                                     PIN_MODE_INPUT(GPIOE_PIN8) | \
                                     PIN_MODE_INPUT(GPIOE_PIN9) | \
                                     PIN_MODE_INPUT(GPIOE_PIN10) | \
                                     PIN_MODE_OUTPUT(GPIOE_SPI4_CS) | \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_SCK) | \
                                     PIN_MODE_OUTPUT(GPIOE_LCD_RSTN) | \
                                     PIN_MODE_ALTERNATE(GPIOE_SPI4_MOSI) | \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_EXT_IO_8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_EXT_IO_0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_ENCODER_TRANSVERS_CHI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_ENCODER_TRANSVERS_CHA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_ENCODER_TRANSVERS_CHB) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_CS) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_SCK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_LCD_RSTN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SPI4_MOSI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(GPIOE_EXT_IO_8) | \
                                     PIN_OSPEED_100M(GPIOE_EXT_IO_0) | \
                                     PIN_OSPEED_100M(GPIOE_PIN2) | \
                                     PIN_OSPEED_100M(GPIOE_PIN3) | \
                                     PIN_OSPEED_100M(GPIOE_ENCODER_TRANSVERS_CHI) | \
                                     PIN_OSPEED_100M(GPIOE_ENCODER_TRANSVERS_CHA) | \
                                     PIN_OSPEED_100M(GPIOE_ENCODER_TRANSVERS_CHB) | \
                                     PIN_OSPEED_100M(GPIOE_PIN7) | \
                                     PIN_OSPEED_100M(GPIOE_PIN8) | \
                                     PIN_OSPEED_100M(GPIOE_PIN9) | \
                                     PIN_OSPEED_100M(GPIOE_PIN10) | \
                                     PIN_OSPEED_100M(GPIOE_SPI4_CS) | \
                                     PIN_OSPEED_100M(GPIOE_SPI4_SCK) | \
                                     PIN_OSPEED_100M(GPIOE_LCD_RSTN) | \
                                     PIN_OSPEED_100M(GPIOE_SPI4_MOSI) | \
                                     PIN_OSPEED_100M(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_FLOATING(GPIOE_EXT_IO_8) | \
                                     PIN_PUPDR_FLOATING(GPIOE_EXT_IO_0) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) | \
                                     PIN_PUPDR_PULLDOWN(GPIOE_ENCODER_TRANSVERS_CHI) | \
                                     PIN_PUPDR_PULLDOWN(GPIOE_ENCODER_TRANSVERS_CHA) | \
                                     PIN_PUPDR_PULLDOWN(GPIOE_ENCODER_TRANSVERS_CHB) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_CS) | \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_SCK) | \
                                     PIN_PUPDR_FLOATING(GPIOE_LCD_RSTN) | \
                                     PIN_PUPDR_FLOATING(GPIOE_SPI4_MOSI) | \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_EXT_IO_8) | \
                                     PIN_ODR_HIGH(GPIOE_EXT_IO_0) | \
                                     PIN_ODR_HIGH(GPIOE_PIN2) | \
                                     PIN_ODR_HIGH(GPIOE_PIN3) | \
                                     PIN_ODR_HIGH(GPIOE_ENCODER_TRANSVERS_CHI) | \
                                     PIN_ODR_HIGH(GPIOE_ENCODER_TRANSVERS_CHA) | \
                                     PIN_ODR_HIGH(GPIOE_ENCODER_TRANSVERS_CHB) | \
                                     PIN_ODR_HIGH(GPIOE_PIN7) | \
                                     PIN_ODR_HIGH(GPIOE_PIN8) | \
                                     PIN_ODR_HIGH(GPIOE_PIN9) | \
                                     PIN_ODR_HIGH(GPIOE_PIN10) | \
                                     PIN_ODR_HIGH(GPIOE_SPI4_CS) | \
                                     PIN_ODR_HIGH(GPIOE_SPI4_SCK) | \
                                     PIN_ODR_HIGH(GPIOE_LCD_RSTN) | \
                                     PIN_ODR_HIGH(GPIOE_SPI4_MOSI) | \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_EXT_IO_8, 0) | \
                                     PIN_AFIO_AF(GPIOE_EXT_IO_0, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOE_ENCODER_TRANSVERS_CHI, 0) | \
                                     PIN_AFIO_AF(GPIOE_ENCODER_TRANSVERS_CHA, 3) | \
                                     PIN_AFIO_AF(GPIOE_ENCODER_TRANSVERS_CHB, 3) | \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOE_SPI4_CS, 0) | \
                                     PIN_AFIO_AF(GPIOE_SPI4_SCK, 5) | \
                                     PIN_AFIO_AF(GPIOE_LCD_RSTN, 5) | \
                                     PIN_AFIO_AF(GPIOE_SPI4_MOSI, 5) | \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) | \
                                     PIN_MODE_INPUT(GPIOF_PIN1) | \
                                     PIN_MODE_INPUT(GPIOF_PIN2) | \
                                     PIN_MODE_INPUT(GPIOF_ROBOT_SELECT) | \
                                     PIN_MODE_INPUT(GPIOF_PIN4) | \
                                     PIN_MODE_INPUT(GPIOF_PIN5) | \
                                     PIN_MODE_ALTERNATE(GPIOF_UART7_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOF_UART7_TX) | \
                                     PIN_MODE_INPUT(GPIOF_PIN8) | \
                                     PIN_MODE_INPUT(GPIOF_EXT_IO_5) | \
                                     PIN_MODE_INPUT(GPIOF_PIN10) | \
                                     PIN_MODE_INPUT(GPIOF_EXT_IO_9) | \
                                     PIN_MODE_INPUT(GPIOF_EXT_IO_7) | \
                                     PIN_MODE_INPUT(GPIOF_PIN13) | \
                                     PIN_MODE_INPUT(GPIOF_PIN14) | \
                                     PIN_MODE_INPUT(GPIOF_EXT_IO_6))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ROBOT_SELECT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_UART7_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_UART7_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_EXT_IO_5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_EXT_IO_9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_EXT_IO_7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_EXT_IO_6))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_PIN0) | \
                                     PIN_OSPEED_100M(GPIOF_PIN1) | \
                                     PIN_OSPEED_100M(GPIOF_PIN2) | \
                                     PIN_OSPEED_100M(GPIOF_ROBOT_SELECT) | \
                                     PIN_OSPEED_100M(GPIOF_PIN4) | \
                                     PIN_OSPEED_100M(GPIOF_PIN5) | \
                                     PIN_OSPEED_100M(GPIOF_UART7_RX) | \
                                     PIN_OSPEED_100M(GPIOF_UART7_TX) | \
                                     PIN_OSPEED_100M(GPIOF_PIN8) | \
                                     PIN_OSPEED_100M(GPIOF_EXT_IO_5) | \
                                     PIN_OSPEED_100M(GPIOF_PIN10) | \
                                     PIN_OSPEED_100M(GPIOF_EXT_IO_9) | \
                                     PIN_OSPEED_100M(GPIOF_EXT_IO_7) | \
                                     PIN_OSPEED_100M(GPIOF_PIN13) | \
                                     PIN_OSPEED_100M(GPIOF_PIN14) | \
                                     PIN_OSPEED_100M(GPIOF_EXT_IO_6))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOF_ROBOT_SELECT) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOF_UART7_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOF_UART7_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOF_EXT_IO_5) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOF_EXT_IO_9) | \
                                     PIN_PUPDR_FLOATING(GPIOF_EXT_IO_7) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOF_EXT_IO_6))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_PIN0) | \
                                     PIN_ODR_LOW(GPIOF_PIN1) | \
                                     PIN_ODR_LOW(GPIOF_PIN2) | \
                                     PIN_ODR_LOW(GPIOF_ROBOT_SELECT) | \
                                     PIN_ODR_LOW(GPIOF_PIN4) | \
                                     PIN_ODR_LOW(GPIOF_PIN5) | \
                                     PIN_ODR_LOW(GPIOF_UART7_RX) | \
                                     PIN_ODR_LOW(GPIOF_UART7_TX) | \
                                     PIN_ODR_LOW(GPIOF_PIN8) | \
                                     PIN_ODR_LOW(GPIOF_EXT_IO_5) | \
                                     PIN_ODR_LOW(GPIOF_PIN10) | \
                                     PIN_ODR_LOW(GPIOF_EXT_IO_9) | \
                                     PIN_ODR_LOW(GPIOF_EXT_IO_7) | \
                                     PIN_ODR_LOW(GPIOF_PIN13) | \
                                     PIN_ODR_LOW(GPIOF_PIN14) | \
                                     PIN_ODR_LOW(GPIOF_EXT_IO_6))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOF_ROBOT_SELECT, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOF_UART7_RX, 8) | \
                                     PIN_AFIO_AF(GPIOF_UART7_TX, 8))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOF_EXT_IO_5, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOF_EXT_IO_9, 0) | \
                                     PIN_AFIO_AF(GPIOF_EXT_IO_7, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOF_EXT_IO_6, 0))

/*
 * GPIOG setup:
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_EXT_IO_1) | \
                                     PIN_MODE_INPUT(GPIOG_EXT_IO_4) | \
                                     PIN_MODE_INPUT(GPIOG_PIN2) | \
                                     PIN_MODE_INPUT(GPIOG_PIN3) | \
                                     PIN_MODE_OUTPUT(GPIOG_LED_RGB_G) | \
                                     PIN_MODE_OUTPUT(GPIOG_LED_RGB_B) | \
                                     PIN_MODE_OUTPUT(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_MODE_INPUT(GPIOG_USB_OVERCURRENT) | \
                                     PIN_MODE_INPUT(GPIOG_EXT_IO_10) | \
                                     PIN_MODE_INPUT(GPIOG_PIN9) | \
                                     PIN_MODE_INPUT(GPIOG_EXT_IO_3) | \
                                     PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_MODE_INPUT(GPIOG_PIN12) | \
                                     PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_MODE_INPUT(GPIOG_PIN14) | \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_EXT_IO_1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_IO_4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LED_RGB_G) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LED_RGB_B) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_OVERCURRENT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_IO_10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_EXT_IO_3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_EXT_IO_1) | \
                                     PIN_OSPEED_100M(GPIOG_EXT_IO_4) | \
                                     PIN_OSPEED_100M(GPIOG_PIN2) | \
                                     PIN_OSPEED_100M(GPIOG_PIN3) | \
                                     PIN_OSPEED_100M(GPIOG_LED_RGB_G) | \
                                     PIN_OSPEED_100M(GPIOG_LED_RGB_B) | \
                                     PIN_OSPEED_100M(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_OSPEED_100M(GPIOG_USB_OVERCURRENT) | \
                                     PIN_OSPEED_100M(GPIOG_EXT_IO_10) | \
                                     PIN_OSPEED_100M(GPIOG_PIN9) | \
                                     PIN_OSPEED_100M(GPIOG_EXT_IO_3) | \
                                     PIN_OSPEED_100M(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_OSPEED_100M(GPIOG_PIN12) | \
                                     PIN_OSPEED_100M(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_OSPEED_100M(GPIOG_PIN14) | \
                                     PIN_OSPEED_100M(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_EXT_IO_1) | \
                                     PIN_PUPDR_FLOATING(GPIOG_EXT_IO_4) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOG_LED_RGB_G) | \
                                     PIN_PUPDR_FLOATING(GPIOG_LED_RGB_B) | \
                                     PIN_PUPDR_FLOATING(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_USB_OVERCURRENT) | \
                                     PIN_PUPDR_FLOATING(GPIOG_EXT_IO_10) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOG_EXT_IO_3) | \
                                     PIN_PUPDR_FLOATING(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_EXT_IO_1) | \
                                     PIN_ODR_LOW(GPIOG_EXT_IO_4) | \
                                     PIN_ODR_LOW(GPIOG_PIN2) | \
                                     PIN_ODR_LOW(GPIOG_PIN3) | \
                                     PIN_ODR_LOW(GPIOG_LED_RGB_G) | \
                                     PIN_ODR_LOW(GPIOG_LED_RGB_B) | \
                                     PIN_ODR_LOW(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_ODR_LOW(GPIOG_USB_OVERCURRENT) | \
                                     PIN_ODR_LOW(GPIOG_EXT_IO_10) | \
                                     PIN_ODR_LOW(GPIOG_PIN9) | \
                                     PIN_ODR_LOW(GPIOG_EXT_IO_3) | \
                                     PIN_ODR_LOW(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_ODR_LOW(GPIOG_PIN12) | \
                                     PIN_ODR_LOW(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_ODR_LOW(GPIOG_PIN14) | \
                                     PIN_ODR_LOW(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_EXT_IO_1, 0) | \
                                     PIN_AFIO_AF(GPIOG_EXT_IO_4, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOG_LED_RGB_G, 0) | \
                                     PIN_AFIO_AF(GPIOG_LED_RGB_B, 0) | \
                                     PIN_AFIO_AF(GPIOG_USB_OTG_POWER_EN, 0) | \
                                     PIN_AFIO_AF(GPIOG_USB_OVERCURRENT, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_EXT_IO_10, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOG_EXT_IO_3, 0) | \
                                     PIN_AFIO_AF(GPIOG_ETH_RMII_TXEN, 11) | \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOG_ETH_RMII_TXD0, 11) | \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

/*
 * GPIOH setup:
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
  void board_led_on(void);
  void board_led_off(void);
  bool board_button_pressed(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
/* clang-format on */
