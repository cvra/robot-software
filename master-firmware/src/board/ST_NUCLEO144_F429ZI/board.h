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

/* todo: flexible switch between 8MHz ST-link MCO (default) and on board 8MHz crystal (optional) */
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
#define GPIOA_ETH_RMII_REF_CLK      1 // RMII_REF_CLK
#define GPIOA_ETH_RMII_MDIO         2 // RMII_MDIO
#define GPIOA_PIN3                  3
#define GPIOA_PIN4                  4
#define GPIOA_PIN5                  5
#define GPIOA_PIN6                  6
#define GPIOA_ETH_RMII_CRS_DV       7 // RMII RX data valid
#define GPIOA_PIN8                  8 // (USB_SOF)
#define GPIOA_USB_VBUS              9 // USB_VBUS
#define GPIOA_OTG_FS_ID             10 // USB_ID
#define GPIOA_OTG_FS_DM             11 // USB_DM
#define GPIOA_OTG_FS_DP             12 // USB_DP
#define GPIOA_SWDIO_JTAG_TMS        13 // SWDIO
#define GPIOA_SWCLK_JTAG_TCK        14 // SWCLK
#define GPIOA_JTAG_TDI              15

#define GPIOB_LED_GREEN             0 // LED_GREEN
#define GPIOB_PIN1                  1
#define GPIOB_BOOT1                 2
#define GPIOB_SWO_JTAG_TDO          3 // SWO
#define GPIOB_JTAG_TRST             4
#define GPIOB_PIN5                  5
#define GPIOB_PIN6                  6
#define GPIOB_LED_BLUE              7 // LED_BLUE
#define GPIOB_PIN8                  8
#define GPIOB_PIN9                  9
#define GPIOB_PIN10                 10
#define GPIOB_PIN11                 11
#define GPIOB_PIN12                 12
#define GPIOB_ETH_RMII_TXD1         13 // RMII_TXD1
#define GPIOB_LED_RED               14 // LED_RED
#define GPIOB_PIN15                 15

#define GPIOC_PIN0                  0
#define GPIOC_ETH_RMII_MDC          1 // RMII_MDC
#define GPIOC_PIN2                  2
#define GPIOC_PIN3                  3
#define GPIOC_ETH_RMII_RXD0         4 // RMII_RXD0
#define GPIOC_ETH_RMII_RXD1         5 // RMII_RXD1
#define GPIOC_ENCODER_RIGHT_CHA     6
#define GPIOC_ENCODER_RIGHT_CHB     7
#define GPIOC_PIN8                  8
#define GPIOC_PIN9                  9
#define GPIOC_PIN10                 10
#define GPIOC_PIN11                 11
#define GPIOC_PIN12                 12
#define GPIOC_USER_BUTTON           13 // USER_BUTTON
#define GPIOC_OSC32_IN              14
#define GPIOC_OSC32_OUT             15

#define GPIOD_CAN1_RX               0
#define GPIOD_CAN1_TX               1
#define GPIOD_PIN2                  2
#define GPIOD_PIN3                  3
#define GPIOD_PIN4                  4
#define GPIOD_PIN5                  5
#define GPIOD_PIN6                  6
#define GPIOD_PIN7                  7
#define GPIOD_UART3_TX              8 // DEBUG_UART_TX
#define GPIOD_UART3_RX              9 // DEBUG_UART_RX
#define GPIOD_PIN10                 10
#define GPIOD_PIN11                 11
#define GPIOD_ENCODER_LEFT_CHA      12 // ENCODER_LEFT_CHA, TIM4_CH1, alternate 2
#define GPIOD_ENCODER_LEFT_CHB      13 // ENCODER_LEFT_CHB, TIM4_CH2, alternate 2
#define GPIOD_PIN14                 14
#define GPIOD_PIN15                 15

#define GPIOE_PIN0                  0
#define GPIOE_PIN1                  1
#define GPIOE_PIN2                  2
#define GPIOE_PIN3                  3
#define GPIOE_ENCODER_LEFT_CHI      4
#define GPIOE_PIN5                  5
#define GPIOE_PIN6                  6
#define GPIOE_PIN7                  7
#define GPIOE_PIN8                  8
#define GPIOE_PIN9                  9
#define GPIOE_PIN10                 10
#define GPIOE_PIN11                 11
#define GPIOE_PIN12                 12
#define GPIOE_PIN13                 13
#define GPIOE_PIN14                 14
#define GPIOE_PIN15                 15

#define GPIOF_LED_READY             0
#define GPIOF_LED_DEBUG             1
#define GPIOF_LED_ERROR             2
#define GPIOF_LED_POWER_ERROR       3
#define GPIOF_LED_PC_ERROR          4
#define GPIOF_LED_BUS_ERROR         5
#define GPIOF_LED_YELLOW_1          6
#define GPIOF_LED_YELLOW_2          7
#define GPIOF_LED_GREEN_1           8
#define GPIOF_LED_GREEN_2           9
#define GPIOF_BTN_YELLOW            10
#define GPIOF_PIN11                 11
#define GPIOF_START                 12
#define GPIOF_BTN_GREEN             13
#define GPIOF_PIN14                 14
#define GPIOF_PIN15                 15

#define GPIOG_PIN0                  0
#define GPIOG_PIN1                  1
#define GPIOG_PIN2                  2
#define GPIOG_PIN3                  3
#define GPIOG_PIN4                  4
#define GPIOG_PIN5                  5
#define GPIOG_USB_OTG_POWER_EN      6 // USB device 5V VBUS power switch
#define GPIOG_USB_OVERCURRENT       7 // USB_OverCurrent, active low (for USB OTG host)
#define GPIOG_PIN8                  8
#define GPIOG_PIN9                  9
#define GPIOG_PIN10                 10
#define GPIOG_ETH_RMII_TXEN         11 // RMII_TX_EN
#define GPIOG_PIN12                 12
#define GPIOG_ETH_RMII_TXD0         13 // RMII_TXD0
#define GPIOG_PIN14                 14
#define GPIOG_ENCODER_RIGHT_CHI     15

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
 *
 * GPIOA_PIN0                       (input floating).
 * GPIOA_ETH_RMII_REF_CLK           (alternate 11).
 * GPIOA_ETH_RMII_MDIO              (alternate 11).
 * GPIOA_PIN3                       (input floating).
 * GPIOA_PIN4                       (input floating).
 * GPIOA_PIN5                       (input floating).
 * GPIOA_PIN6                       (input floating).
 * GPIOA_ETH_RMII_CRS_DV            (alternate 11).
 * GPIOA_PIN8                       (input floating).
 * GPIOA_USB_VBUS                   (input pulldown).
 * GPIOA_OTG_FS_ID                  (alternate 10).
 * GPIOA_OTG_FS_DM                  (alternate 10).
 * GPIOA_OTG_FS_DP                  (alternate 10).
 * GPIOA_SWDIO_JTAG_TMS             (alternate 0).
 * GPIOA_SWCLK_JTAG_TCK             (alternate 0).
 * GPIOA_JTAG_TDI                   (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_PIN0) | \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_REF_CLK) | \
                                     PIN_MODE_ALTERNATE(GPIOA_ETH_RMII_MDIO) | \
                                     PIN_MODE_INPUT(GPIOA_PIN3) | \
                                     PIN_MODE_INPUT(GPIOA_PIN4) | \
                                     PIN_MODE_INPUT(GPIOA_PIN5) | \
                                     PIN_MODE_INPUT(GPIOA_PIN6) | \
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
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN6) | \
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
                                     PIN_OSPEED_100M(GPIOA_PIN5) | \
                                     PIN_OSPEED_100M(GPIOA_PIN6) | \
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
                                     PIN_PUPDR_FLOATING(GPIOA_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN6) | \
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
                                     PIN_ODR_HIGH(GPIOA_PIN5) | \
                                     PIN_ODR_HIGH(GPIOA_PIN6) | \
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
                                     PIN_AFIO_AF(GPIOA_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOA_PIN6, 0) | \
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
 *
 * GPIOB_LED_GREEN                  (output pushpull max).
 * GPIOB_PIN1                       (input floating).
 * GPIOB_BOOT1                      (input floating).
 * GPIOB_SWO_JTAG_TDO               (alternate 0).
 * GPIOB_JTAG_TRST                  (alternate 0).
 * GPIOB_PIN5                       (input floating).
 * GPIOB_PIN6                       (input floating).
 * GPIOB_LED_BLUE                   (output pushpull max).
 * GPIOB_PIN8                       (input floating).
 * GPIOB_PIN9                       (input floating).
 * GPIOB_PIN10                      (input floating).
 * GPIOB_PIN11                      (input floating).
 * GPIOB_PIN12                      (input floating).
 * GPIOB_ETH_RMII_TXD1              (alternate 11).
 * GPIOB_LED_RED                    (output pushpull max).
 * GPIOB_PIN15                      (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_LED_GREEN) | \
                                     PIN_MODE_INPUT(GPIOB_PIN1) | \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TRST) | \
                                     PIN_MODE_INPUT(GPIOB_PIN5) | \
                                     PIN_MODE_INPUT(GPIOB_PIN6) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_BLUE) | \
                                     PIN_MODE_INPUT(GPIOB_PIN8) | \
                                     PIN_MODE_INPUT(GPIOB_PIN9) | \
                                     PIN_MODE_INPUT(GPIOB_PIN10) | \
                                     PIN_MODE_INPUT(GPIOB_PIN11) | \
                                     PIN_MODE_INPUT(GPIOB_PIN12) | \
                                     PIN_MODE_ALTERNATE(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_MODE_OUTPUT(GPIOB_LED_RED) | \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LED_GREEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TRST) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_BLUE) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LED_RED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN15))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_100M(GPIOB_LED_GREEN) | \
                                     PIN_OSPEED_100M(GPIOB_PIN1) | \
                                     PIN_OSPEED_100M(GPIOB_BOOT1) | \
                                     PIN_OSPEED_100M(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_OSPEED_100M(GPIOB_JTAG_TRST) | \
                                     PIN_OSPEED_100M(GPIOB_PIN5) | \
                                     PIN_OSPEED_100M(GPIOB_PIN6) | \
                                     PIN_OSPEED_100M(GPIOB_LED_BLUE) | \
                                     PIN_OSPEED_100M(GPIOB_PIN8) | \
                                     PIN_OSPEED_100M(GPIOB_PIN9) | \
                                     PIN_OSPEED_100M(GPIOB_PIN10) | \
                                     PIN_OSPEED_100M(GPIOB_PIN11) | \
                                     PIN_OSPEED_100M(GPIOB_PIN12) | \
                                     PIN_OSPEED_100M(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_OSPEED_100M(GPIOB_LED_RED) | \
                                     PIN_OSPEED_100M(GPIOB_PIN15))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LED_GREEN) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_PUPDR_PULLUP(GPIOB_JTAG_TRST) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_BLUE) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_PUPDR_FLOATING(GPIOB_LED_RED) | \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN15))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_LED_GREEN) | \
                                     PIN_ODR_LOW(GPIOB_PIN1) | \
                                     PIN_ODR_LOW(GPIOB_BOOT1) | \
                                     PIN_ODR_LOW(GPIOB_SWO_JTAG_TDO) | \
                                     PIN_ODR_LOW(GPIOB_JTAG_TRST) | \
                                     PIN_ODR_LOW(GPIOB_PIN5) | \
                                     PIN_ODR_LOW(GPIOB_PIN6) | \
                                     PIN_ODR_LOW(GPIOB_LED_BLUE) | \
                                     PIN_ODR_LOW(GPIOB_PIN8) | \
                                     PIN_ODR_LOW(GPIOB_PIN9) | \
                                     PIN_ODR_LOW(GPIOB_PIN10) | \
                                     PIN_ODR_LOW(GPIOB_PIN11) | \
                                     PIN_ODR_LOW(GPIOB_PIN12) | \
                                     PIN_ODR_LOW(GPIOB_ETH_RMII_TXD1) | \
                                     PIN_ODR_LOW(GPIOB_LED_RED) | \
                                     PIN_ODR_LOW(GPIOB_PIN15))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LED_GREEN, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) | \
                                     PIN_AFIO_AF(GPIOB_SWO_JTAG_TDO, 0) | \
                                     PIN_AFIO_AF(GPIOB_JTAG_TRST, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOB_LED_BLUE, 0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOB_ETH_RMII_TXD1, 11) | \
                                     PIN_AFIO_AF(GPIOB_LED_RED, 0) | \
                                     PIN_AFIO_AF(GPIOB_PIN15, 0))

/*
 * GPIOC setup:
 *
 * GPIOC_PIN0                       (input floating).
 * GPIOC_ETH_RMII_MDC               (alternate 11).
 * GPIOC_PIN2                       (input floating).
 * GPIOC_PIN3                       (input floating).
 * GPIOC_ETH_RMII_RXD0              (alternate 11).
 * GPIOC_ETH_RMII_RXD1              (alternate 11).
 * GPIOC_ENCODER_RIGHT_CHA          (alternate 2).
 * GPIOC_ENCODER_RIGHT_CHB          (alternate 2).
 * GPIOC_PIN8                       (input floating).
 * GPIOC_PIN9                       (input floating).
 * GPIOC_PIN10                      (input floating).
 * GPIOC_PIN11                      (input floating).
 * GPIOC_PIN12                      (input floating).
 * GPIOC_USER_BUTTON                (input floating).
 * GPIOC_OSC32_IN                   (input floating).
 * GPIOC_OSC32_OUT                  (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(GPIOC_PIN0) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_MDC) | \
                                     PIN_MODE_INPUT(GPIOC_PIN2) | \
                                     PIN_MODE_INPUT(GPIOC_PIN3) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_MODE_ALTERNATE(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_MODE_INPUT(GPIOC_PIN8) | \
                                     PIN_MODE_INPUT(GPIOC_PIN9) | \
                                     PIN_MODE_INPUT(GPIOC_PIN10) | \
                                     PIN_MODE_INPUT(GPIOC_PIN11) | \
                                     PIN_MODE_INPUT(GPIOC_PIN12) | \
                                     PIN_MODE_INPUT(GPIOC_USER_BUTTON) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_MDC) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USER_BUTTON) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(GPIOC_PIN0) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_MDC) | \
                                     PIN_OSPEED_100M(GPIOC_PIN2) | \
                                     PIN_OSPEED_100M(GPIOC_PIN3) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_OSPEED_100M(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_OSPEED_100M(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_OSPEED_100M(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_OSPEED_100M(GPIOC_PIN8) | \
                                     PIN_OSPEED_100M(GPIOC_PIN9) | \
                                     PIN_OSPEED_100M(GPIOC_PIN10) | \
                                     PIN_OSPEED_100M(GPIOC_PIN11) | \
                                     PIN_OSPEED_100M(GPIOC_PIN12) | \
                                     PIN_OSPEED_100M(GPIOC_USER_BUTTON) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_MDC) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_PUPDR_FLOATING(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOC_USER_BUTTON) | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN0) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_MDC) | \
                                     PIN_ODR_LOW(GPIOC_PIN2) | \
                                     PIN_ODR_LOW(GPIOC_PIN3) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD0) | \
                                     PIN_ODR_LOW(GPIOC_ETH_RMII_RXD1) | \
                                     PIN_ODR_LOW(GPIOC_ENCODER_RIGHT_CHA) | \
                                     PIN_ODR_LOW(GPIOC_ENCODER_RIGHT_CHB) | \
                                     PIN_ODR_LOW(GPIOC_PIN8) | \
                                     PIN_ODR_LOW(GPIOC_PIN9) | \
                                     PIN_ODR_LOW(GPIOC_PIN10) | \
                                     PIN_ODR_LOW(GPIOC_PIN11) | \
                                     PIN_ODR_LOW(GPIOC_PIN12) | \
                                     PIN_ODR_LOW(GPIOC_USER_BUTTON) | \
                                     PIN_ODR_LOW(GPIOC_OSC32_IN) | \
                                     PIN_ODR_LOW(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_MDC, 11) | \
                                     PIN_AFIO_AF(GPIOC_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_RXD0, 11) | \
                                     PIN_AFIO_AF(GPIOC_ETH_RMII_RXD1, 11) | \
                                     PIN_AFIO_AF(GPIOC_ENCODER_RIGHT_CHA, 2) | \
                                     PIN_AFIO_AF(GPIOC_ENCODER_RIGHT_CHB, 2))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOC_USER_BUTTON, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
 * GPIOD_CAN1_RX                    (alternate 9).
 * GPIOD_CAN1_TX                    (alternate 9).
 * GPIOD_PIN2                       (input floating).
 * GPIOD_PIN3                       (input floating).
 * GPIOD_PIN4                       (input floating).
 * GPIOD_PIN5                       (input floating).
 * GPIOD_PIN6                       (input floating).
 * GPIOD_PIN7                       (input floating).
 * GPIOD_UART3_TX                   (alternate 7).
 * GPIOD_UART3_RX                   (alternate 7, pullup).
 * GPIOD_PIN10                      (input floating).
 * GPIOD_PIN11                      (input floating).
 * GPIOD_ENCODER_LEFT_CHA           (alternate 2).
 * GPIOD_ENCODER_LEFT_CHB           (alternate 2).
 * GPIOD_PIN14                      (input pullup).
 * GPIOD_PIN15                      (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(GPIOD_CAN1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOD_CAN1_TX) | \
                                     PIN_MODE_INPUT(GPIOD_PIN2) | \
                                     PIN_MODE_INPUT(GPIOD_PIN3) | \
                                     PIN_MODE_INPUT(GPIOD_PIN4) | \
                                     PIN_MODE_INPUT(GPIOD_PIN5) | \
                                     PIN_MODE_INPUT(GPIOD_PIN6) | \
                                     PIN_MODE_INPUT(GPIOD_PIN7) | \
                                     PIN_MODE_ALTERNATE(GPIOD_UART3_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOD_UART3_RX) | \
                                     PIN_MODE_INPUT(GPIOD_PIN10) | \
                                     PIN_MODE_INPUT(GPIOD_PIN11) | \
                                     PIN_MODE_ALTERNATE(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_MODE_ALTERNATE(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_MODE_INPUT(GPIOD_PIN14) | \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_CAN1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CAN1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART3_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_UART3_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(GPIOD_CAN1_RX) | \
                                     PIN_OSPEED_100M(GPIOD_CAN1_TX) | \
                                     PIN_OSPEED_100M(GPIOD_PIN2) | \
                                     PIN_OSPEED_100M(GPIOD_PIN3) | \
                                     PIN_OSPEED_100M(GPIOD_PIN4) | \
                                     PIN_OSPEED_100M(GPIOD_PIN5) | \
                                     PIN_OSPEED_100M(GPIOD_PIN6) | \
                                     PIN_OSPEED_100M(GPIOD_PIN7) | \
                                     PIN_OSPEED_100M(GPIOD_UART3_TX) | \
                                     PIN_OSPEED_100M(GPIOD_UART3_RX) | \
                                     PIN_OSPEED_100M(GPIOD_PIN10) | \
                                     PIN_OSPEED_100M(GPIOD_PIN11) | \
                                     PIN_OSPEED_100M(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_OSPEED_100M(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_OSPEED_100M(GPIOD_PIN14) | \
                                     PIN_OSPEED_100M(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_CAN1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOD_CAN1_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN6) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) | \
                                     PIN_PUPDR_FLOATING(GPIOD_UART3_TX) | \
                                     PIN_PUPDR_PULLUP(GPIOD_UART3_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_PUPDR_FLOATING(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_LOW(GPIOD_CAN1_RX) | \
                                     PIN_ODR_LOW(GPIOD_CAN1_TX) | \
                                     PIN_ODR_LOW(GPIOD_PIN2) | \
                                     PIN_ODR_LOW(GPIOD_PIN3) | \
                                     PIN_ODR_LOW(GPIOD_PIN4) | \
                                     PIN_ODR_LOW(GPIOD_PIN5) | \
                                     PIN_ODR_LOW(GPIOD_PIN6) | \
                                     PIN_ODR_LOW(GPIOD_PIN7) | \
                                     PIN_ODR_LOW(GPIOD_UART3_TX) | \
                                     PIN_ODR_LOW(GPIOD_UART3_RX) | \
                                     PIN_ODR_LOW(GPIOD_PIN10) | \
                                     PIN_ODR_LOW(GPIOD_PIN11) | \
                                     PIN_ODR_LOW(GPIOD_ENCODER_LEFT_CHA) | \
                                     PIN_ODR_LOW(GPIOD_ENCODER_LEFT_CHB) | \
                                     PIN_ODR_LOW(GPIOD_PIN14) | \
                                     PIN_ODR_LOW(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_CAN1_RX, 9) | \
                                     PIN_AFIO_AF(GPIOD_CAN1_TX, 9) | \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_UART3_TX, 7) | \
                                     PIN_AFIO_AF(GPIOD_UART3_RX, 7) | \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOD_ENCODER_LEFT_CHA, 2) | \
                                     PIN_AFIO_AF(GPIOD_ENCODER_LEFT_CHB, 2) | \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * GPIOE_PIN0                       (input pullup).
 * GPIOE_PIN1                       (input pullup).
 * GPIOE_PIN2                       (input pullup).
 * GPIOE_PIN3                       (input pullup).
 * GPIOE_ENCODER_LEFT_CHI           (input pullup).
 * GPIOE_PIN5                       (input pullup).
 * GPIOE_PIN6                       (input pullup).
 * GPIOE_PIN7                       (input pullup).
 * GPIOE_PIN8                       (input pullup).
 * GPIOE_PIN9                       (input pullup).
 * GPIOE_PIN10                      (input pullup).
 * GPIOE_PIN11                      (input pullup).
 * GPIOE_PIN12                      (input pullup).
 * GPIOE_PIN13                      (input pullup).
 * GPIOE_PIN14                      (input pullup).
 * GPIOE_PIN15                      (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) | \
                                     PIN_MODE_INPUT(GPIOE_PIN1) | \
                                     PIN_MODE_INPUT(GPIOE_PIN2) | \
                                     PIN_MODE_INPUT(GPIOE_PIN3) | \
                                     PIN_MODE_INPUT(GPIOE_ENCODER_LEFT_CHI) | \
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
                                     PIN_OTYPE_PUSHPULL(GPIOE_ENCODER_LEFT_CHI) | \
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
                                     PIN_OSPEED_100M(GPIOE_ENCODER_LEFT_CHI) | \
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
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) | \
                                     PIN_PUPDR_PULLUP(GPIOE_ENCODER_LEFT_CHI) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) | \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) | \
                                     PIN_ODR_HIGH(GPIOE_PIN1) | \
                                     PIN_ODR_HIGH(GPIOE_PIN2) | \
                                     PIN_ODR_HIGH(GPIOE_PIN3) | \
                                     PIN_ODR_HIGH(GPIOE_ENCODER_LEFT_CHI) | \
                                     PIN_ODR_HIGH(GPIOE_PIN5) | \
                                     PIN_ODR_HIGH(GPIOE_PIN6) | \
                                     PIN_ODR_HIGH(GPIOE_PIN7) | \
                                     PIN_ODR_HIGH(GPIOE_PIN8) | \
                                     PIN_ODR_HIGH(GPIOE_PIN9) | \
                                     PIN_ODR_HIGH(GPIOE_PIN10) | \
                                     PIN_ODR_HIGH(GPIOE_PIN11) | \
                                     PIN_ODR_HIGH(GPIOE_PIN12) | \
                                     PIN_ODR_HIGH(GPIOE_PIN13) | \
                                     PIN_ODR_HIGH(GPIOE_PIN14) | \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOE_ENCODER_LEFT_CHI, 0) | \
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
 * GPIOF_LED_READY                  (output).
 * GPIOF_LED_DEBUG                  (output).
 * GPIOF_LED_ERROR                  (output).
 * GPIOF_LED_POWER_ERROR            (output).
 * GPIOF_LED_PC_ERROR               (output).
 * GPIOF_LED_BUS_ERROR              (output).
 * GPIOF_LED_YELLOW_1               (output).
 * GPIOF_LED_YELLOW_2               (output).
 * GPIOF_LED_GREEN_1                (output).
 * GPIOF_LED_GREEN_2                (output).
 * GPIOF_BTN_YELLOW                 (input pullup).
 * GPIOF_PIN11                      (input floating).
 * GPIOF_START                      (input pullup).
 * GPIOF_BTN_GREEN                  (input pullup).
 * GPIOF_PIN14                      (input floating).
 * GPIOF_PIN15                      (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_OUTPUT(GPIOF_LED_READY) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_DEBUG) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_ERROR) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_POWER_ERROR) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_PC_ERROR) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_BUS_ERROR) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_YELLOW_1) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_YELLOW_2) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_GREEN_1) | \
                                     PIN_MODE_OUTPUT(GPIOF_LED_GREEN_2) | \
                                     PIN_MODE_INPUT(GPIOF_BTN_YELLOW) | \
                                     PIN_MODE_INPUT(GPIOF_PIN11) | \
                                     PIN_MODE_INPUT(GPIOF_START) | \
                                     PIN_MODE_INPUT(GPIOF_BTN_GREEN) | \
                                     PIN_MODE_INPUT(GPIOF_PIN14) | \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_LED_READY) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_DEBUG) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_ERROR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_POWER_ERROR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_PC_ERROR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_BUS_ERROR) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_YELLOW_1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_YELLOW_2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_GREEN_1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_GREEN_2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_BTN_YELLOW) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_START) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_BTN_GREEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(GPIOF_LED_READY) | \
                                     PIN_OSPEED_100M(GPIOF_LED_DEBUG) | \
                                     PIN_OSPEED_100M(GPIOF_LED_ERROR) | \
                                     PIN_OSPEED_100M(GPIOF_LED_POWER_ERROR) | \
                                     PIN_OSPEED_100M(GPIOF_LED_PC_ERROR) | \
                                     PIN_OSPEED_100M(GPIOF_LED_BUS_ERROR) | \
                                     PIN_OSPEED_100M(GPIOF_LED_YELLOW_1) | \
                                     PIN_OSPEED_100M(GPIOF_LED_YELLOW_2) | \
                                     PIN_OSPEED_100M(GPIOF_LED_GREEN_1) | \
                                     PIN_OSPEED_100M(GPIOF_LED_GREEN_2) | \
                                     PIN_OSPEED_100M(GPIOF_BTN_YELLOW) | \
                                     PIN_OSPEED_100M(GPIOF_PIN11) | \
                                     PIN_OSPEED_100M(GPIOF_START) | \
                                     PIN_OSPEED_100M(GPIOF_BTN_GREEN) | \
                                     PIN_OSPEED_100M(GPIOF_PIN14) | \
                                     PIN_OSPEED_100M(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_LED_READY) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_DEBUG) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_ERROR) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_POWER_ERROR) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_PC_ERROR) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_BUS_ERROR) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_YELLOW_1) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_YELLOW_2) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_GREEN_1) | \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_GREEN_2) | \
                                     PIN_PUPDR_PULLUP(GPIOF_BTN_YELLOW) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) | \
                                     PIN_PUPDR_PULLUP(GPIOF_START) | \
                                     PIN_PUPDR_PULLUP(GPIOF_BTN_GREEN) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) | \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_LOW(GPIOF_LED_READY) | \
                                     PIN_ODR_LOW(GPIOF_LED_DEBUG) | \
                                     PIN_ODR_LOW(GPIOF_LED_ERROR) | \
                                     PIN_ODR_LOW(GPIOF_LED_POWER_ERROR) | \
                                     PIN_ODR_LOW(GPIOF_LED_PC_ERROR) | \
                                     PIN_ODR_LOW(GPIOF_LED_BUS_ERROR) | \
                                     PIN_ODR_LOW(GPIOF_LED_YELLOW_1) | \
                                     PIN_ODR_LOW(GPIOF_LED_YELLOW_2) | \
                                     PIN_ODR_LOW(GPIOF_LED_GREEN_1) | \
                                     PIN_ODR_LOW(GPIOF_LED_GREEN_2) | \
                                     PIN_ODR_LOW(GPIOF_BTN_YELLOW) | \
                                     PIN_ODR_LOW(GPIOF_PIN11) | \
                                     PIN_ODR_LOW(GPIOF_START) | \
                                     PIN_ODR_LOW(GPIOF_BTN_GREEN) | \
                                     PIN_ODR_LOW(GPIOF_PIN14) | \
                                     PIN_ODR_LOW(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_LED_READY, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_DEBUG, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_ERROR, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_POWER_ERROR, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_PC_ERROR, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_BUS_ERROR, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_YELLOW_1, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_YELLOW_2, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_LED_GREEN_1, 0) | \
                                     PIN_AFIO_AF(GPIOF_LED_GREEN_2, 0) | \
                                     PIN_AFIO_AF(GPIOF_BTN_YELLOW, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) | \
                                     PIN_AFIO_AF(GPIOF_START, 0) | \
                                     PIN_AFIO_AF(GPIOF_BTN_GREEN, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

/*
 * GPIOG setup:
 *
 * GPIOG_PIN0                       (input floating).
 * GPIOG_PIN1                       (input floating).
 * GPIOG_PIN2                       (input floating).
 * GPIOG_PIN3                       (input floating).
 * GPIOG_PIN4                       (input floating).
 * GPIOG_PIN5                       (input floating).
 * GPIOG_USB_OTG_POWER_EN           (output low).
 * GPIOG_USB_OVERCURRENT            (input floating).
 * GPIOG_PIN8                       (input floating).
 * GPIOG_PIN9                       (input floating).
 * GPIOG_PIN10                      (input floating).
 * GPIOG_ETH_RMII_TXEN              (alternate 11).
 * GPIOG_PIN12                      (input floating).
 * GPIOG_ETH_RMII_TXD0              (alternate 11).
 * GPIOG_PIN14                      (input floating).
 * GPIOG_ENCODER_RIGHT_CHI          (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) | \
                                     PIN_MODE_INPUT(GPIOG_PIN1) | \
                                     PIN_MODE_INPUT(GPIOG_PIN2) | \
                                     PIN_MODE_INPUT(GPIOG_PIN3) | \
                                     PIN_MODE_INPUT(GPIOG_PIN4) | \
                                     PIN_MODE_INPUT(GPIOG_PIN5) | \
                                     PIN_MODE_OUTPUT(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_MODE_INPUT(GPIOG_USB_OVERCURRENT) | \
                                     PIN_MODE_INPUT(GPIOG_PIN8) | \
                                     PIN_MODE_INPUT(GPIOG_PIN9) | \
                                     PIN_MODE_INPUT(GPIOG_PIN10) | \
                                     PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_MODE_INPUT(GPIOG_PIN12) | \
                                     PIN_MODE_ALTERNATE(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_MODE_INPUT(GPIOG_PIN14) | \
                                     PIN_MODE_INPUT(GPIOG_ENCODER_RIGHT_CHI))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_USB_OVERCURRENT) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_ENCODER_RIGHT_CHI))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_100M(GPIOG_PIN0) | \
                                     PIN_OSPEED_100M(GPIOG_PIN1) | \
                                     PIN_OSPEED_100M(GPIOG_PIN2) | \
                                     PIN_OSPEED_100M(GPIOG_PIN3) | \
                                     PIN_OSPEED_100M(GPIOG_PIN4) | \
                                     PIN_OSPEED_100M(GPIOG_PIN5) | \
                                     PIN_OSPEED_100M(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_OSPEED_100M(GPIOG_USB_OVERCURRENT) | \
                                     PIN_OSPEED_100M(GPIOG_PIN8) | \
                                     PIN_OSPEED_100M(GPIOG_PIN9) | \
                                     PIN_OSPEED_100M(GPIOG_PIN10) | \
                                     PIN_OSPEED_100M(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_OSPEED_100M(GPIOG_PIN12) | \
                                     PIN_OSPEED_100M(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_OSPEED_100M(GPIOG_PIN14) | \
                                     PIN_OSPEED_100M(GPIOG_ENCODER_RIGHT_CHI))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_PIN0) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN1) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN2) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN4) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_USB_OVERCURRENT) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN9) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) | \
                                     PIN_PUPDR_FLOATING(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) | \
                                     PIN_PUPDR_FLOATING(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN14) | \
                                     PIN_PUPDR_PULLUP(GPIOG_ENCODER_RIGHT_CHI))
#define VAL_GPIOG_ODR               (PIN_ODR_LOW(GPIOG_PIN0) | \
                                     PIN_ODR_LOW(GPIOG_PIN1) | \
                                     PIN_ODR_LOW(GPIOG_PIN2) | \
                                     PIN_ODR_LOW(GPIOG_PIN3) | \
                                     PIN_ODR_LOW(GPIOG_PIN4) | \
                                     PIN_ODR_LOW(GPIOG_PIN5) | \
                                     PIN_ODR_LOW(GPIOG_USB_OTG_POWER_EN) | \
                                     PIN_ODR_LOW(GPIOG_USB_OVERCURRENT) | \
                                     PIN_ODR_LOW(GPIOG_PIN8) | \
                                     PIN_ODR_LOW(GPIOG_PIN9) | \
                                     PIN_ODR_LOW(GPIOG_PIN10) | \
                                     PIN_ODR_LOW(GPIOG_ETH_RMII_TXEN) | \
                                     PIN_ODR_LOW(GPIOG_PIN12) | \
                                     PIN_ODR_LOW(GPIOG_ETH_RMII_TXD0) | \
                                     PIN_ODR_LOW(GPIOG_PIN14) | \
                                     PIN_ODR_LOW(GPIOG_ENCODER_RIGHT_CHI))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) | \
                                     PIN_AFIO_AF(GPIOG_USB_OTG_POWER_EN, 0) | \
                                     PIN_AFIO_AF(GPIOG_USB_OVERCURRENT, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) | \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) | \
                                     PIN_AFIO_AF(GPIOG_ETH_RMII_TXEN, 11) | \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) | \
                                     PIN_AFIO_AF(GPIOG_ETH_RMII_TXD0, 11) | \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) | \
                                     PIN_AFIO_AF(GPIOG_ENCODER_RIGHT_CHI, 0))

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
  void board_led_on(void);
  void board_led_off(void);
  bool board_button_pressed(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
