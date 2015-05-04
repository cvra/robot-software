#include <hal.h>
#include "interface_panel.h"

/*
 * pin mapping
 * -----------
 * PF0:  LED ready
 * PF1:  LED debug
 * PF2:  LED error
 * PF3:  LED power error
 * PF4:  LED pc error
 * PF5:  LED bus error
 * PF6:  LED yellow 1
 * PF7:  LED yellow 2
 * PF8:  LED green 1
 * PF9:  LED green 2
 * PF10: button yellow
 * PF12: start
 * PF13: button green
 *
 * leds are active high
 * buttons are active low with MCU pullup
 * start is active high with MCU pullup
 *
 * gpio setup is done in board.h
 */


void interface_panel_test(void)
{
    // press both buttons to exit
    while (palReadPad(GPIOF, GPIOF_BTN_GREEN) || palReadPad(GPIOF, GPIOF_BTN_YELLOW)) {
        if (!palReadPad(GPIOF, GPIOF_BTN_GREEN)) {
            palSetPad(GPIOF, GPIOF_LED_GREEN_1);
            palSetPad(GPIOF, GPIOF_LED_GREEN_2);
        } else {
            palClearPad(GPIOF, GPIOF_LED_GREEN_1);
            palClearPad(GPIOF, GPIOF_LED_GREEN_2);
        }
        if (!palReadPad(GPIOF, GPIOF_BTN_YELLOW)) {
            palSetPad(GPIOF, GPIOF_LED_YELLOW_1);
            palSetPad(GPIOF, GPIOF_LED_YELLOW_2);
        } else {
            palClearPad(GPIOF, GPIOF_LED_YELLOW_1);
            palClearPad(GPIOF, GPIOF_LED_YELLOW_2);
        }
        if (palReadPad(GPIOF, GPIOF_START)) {
            palSetPad(GPIOF, GPIOF_LED_READY);
            palSetPad(GPIOF, GPIOF_LED_DEBUG);
            palClearPad(GPIOF, GPIOF_LED_ERROR);
            palClearPad(GPIOF, GPIOF_LED_POWER_ERROR);
            palClearPad(GPIOF, GPIOF_LED_PC_ERROR);
            palClearPad(GPIOF, GPIOF_LED_BUS_ERROR);
        } else {
            palClearPad(GPIOF, GPIOF_LED_READY);
            palClearPad(GPIOF, GPIOF_LED_DEBUG);
            palSetPad(GPIOF, GPIOF_LED_ERROR);
            palSetPad(GPIOF, GPIOF_LED_POWER_ERROR);
            palSetPad(GPIOF, GPIOF_LED_PC_ERROR);
            palSetPad(GPIOF, GPIOF_LED_BUS_ERROR);
        }
        chThdSleepMilliseconds(100);
    }
    palClearPad(GPIOF, GPIOF_LED_GREEN_1);
    palClearPad(GPIOF, GPIOF_LED_GREEN_2);
    palClearPad(GPIOF, GPIOF_LED_YELLOW_1);
    palClearPad(GPIOF, GPIOF_LED_YELLOW_2);
    palClearPad(GPIOF, GPIOF_LED_DEBUG);
    palClearPad(GPIOF, GPIOF_LED_ERROR);
    palClearPad(GPIOF, GPIOF_LED_POWER_ERROR);
    palClearPad(GPIOF, GPIOF_LED_PC_ERROR);
    palClearPad(GPIOF, GPIOF_LED_BUS_ERROR);
    palSetPad(GPIOF, GPIOF_LED_READY);
}

