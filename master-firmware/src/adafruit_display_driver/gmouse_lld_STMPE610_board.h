/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.org/license.html
 */

#ifndef _GINPUT_LLD_MOUSE_BOARD_H
#define _GINPUT_LLD_MOUSE_BOARD_H

#include <error/error.h>

// Resolution and Accuracy Settings
#define GMOUSE_STMPE610_PEN_CALIBRATE_ERROR    8
#define GMOUSE_STMPE610_PEN_CLICK_ERROR        6
#define GMOUSE_STMPE610_PEN_MOVE_ERROR         4
#define GMOUSE_STMPE610_FINGER_CALIBRATE_ERROR 14
#define GMOUSE_STMPE610_FINGER_CLICK_ERROR     18
#define GMOUSE_STMPE610_FINGER_MOVE_ERROR      14

// How much extra data to allocate at the end of the GMouse structure for the board's use
#define GMOUSE_STMPE610_BOARD_DATA_SIZE        0

// Options - Leave these commented to make it user configurable in the gfxconf.h
// #define GMOUSE_STMPE610_READ_PRESSURE		FALSE
// #define GMOUSE_STMPE610_SELF_CALIBRATE	FALSE
// #define GMOUSE_STMPE610_TEST_MODE			FALSE

// If TRUE this board has the STMPE610 IRQ pin connected to a GPIO.
// Note: For tested hardware this is unreliable and should be set to FALSE until tested.
//			Symptoms are that mouse readings just appear to stop for a bit. Lifting the touch
//			and re-applying the touch cause readings to start again.
#define GMOUSE_STMPE610_GPIO_IRQPIN FALSE

// If TRUE this is a really slow CPU and we should always clear the FIFO between reads.
#define GMOUSE_STMPE610_SLOW_CPU    FALSE

#define STMPE610_I2C_ADDR 0x41

static bool_t init_board(GMouse* m, unsigned driverinstance)
{
    static I2CConfig config;
    config.op_mode = OPMODE_I2C;
    config.clock_speed = 100 * 1000;
    config.duty_cycle = STD_DUTY_CYCLE;

    i2cStart(&I2CD2, &config);

    return true;
}

static void aquire_bus(GMouse* m)
{
 //   i2cAcquireBus(&I2CD2);
}

static void release_bus(GMouse* m)
{
//    i2cReleaseBus(&I2CD2);
}

static void write_reg(GMouse* m, uint8_t reg, uint8_t val)
{
    uint8_t buffer[2] = {reg, val};
    i2cMasterTransmitTimeout(&I2CD2, STMPE610_I2C_ADDR, buffer, sizeof(buffer),
                                         NULL, 0, MS2ST(10));
}

static uint8_t read_byte(GMouse* m, uint8_t reg)
{
    uint8_t reply;
    i2cMasterTransmitTimeout(&I2CD2, STMPE610_I2C_ADDR, &reg, sizeof(reg),
                                         &reply, sizeof(reply), MS2ST(10));

    return reply;
}

static uint16_t read_word(GMouse* m, uint8_t reg)
{
    uint8_t reply[2];
    i2cMasterTransmitTimeout(&I2CD2, 0x41, &reg, sizeof(reg), &reply, sizeof(reply), MS2ST(10));
    uint16_t result;
    result = reply[0];
    result <<= 8;
    result |= reply[1];
    return result;
}

#endif /* _GINPUT_LLD_MOUSE_BOARD_H */
