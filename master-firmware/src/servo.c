#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include "servo.h"

/* The PCA9685 PWM LED controller is used as a servo controller */
#define PCA9685_CLOCK           25000000 // Hz
#define PWM_PRESCALER           125
#define PWM_CLOCK               200000 // Hz
// PWM counter max: 4096 -> 20.48ms Period

#define PCA9685_I2C_ADDR        0b0100000

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_LED0_OFF_L      0x08
#define PCA9685_PRE_SCALE       0xFE

#define PCA9685_MODE1_AI        (1 << 5)
#define PCA9685_MODE2_OUTDRV    (1 << 2)
#define PCA9685_MODE2_OCH       (1 << 3)

#define SERVO_COUNT 16

static void pca9685_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, buf, 2, NULL, 0);
}

// static uint8_t pca9685_read_reg(uint8_t reg_addr)
// {
//     uint8_t data;
//     i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, &reg_addr, 1, &data, 1);
//     return data;
// }

void servo_set(unsigned int servo_nb, float pos)
{
    if (servo_nb >= SERVO_COUNT) {
        WARNING("Servo %u out of range", servo_nb);
        return;
    }

    if (pos > 1.0f) {
        pos = 1.0f;
    } else if (pos < 0.0f) {
        pos = 0.0f;
    }

    // 1 - 2ms pulse length
    uint16_t count = (uint16_t)((pos + 1.0f) * (PWM_CLOCK / 1000));

    uint8_t buf[3] = {
        PCA9685_LED0_OFF_L + 4 * servo_nb,
        (uint8_t) count,
        (uint8_t) (count >> 8)
    };

    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, buf, sizeof(buf), NULL, 0);
    i2cReleaseBus(&I2CD1);
}

void servo_init(void)
{
    palSetPadMode(GPIOB, GPIOB_I2C1_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(GPIOB, GPIOB_I2C1_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    // output disable
    palSetPadMode(GPIOC, GPIOC_SERVO_OUTPUT_EN, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(GPIOC, GPIOC_SERVO_OUTPUT_EN);

    static const I2CConfig i2c_config = {
        OPMODE_I2C,
        100000,
        STD_DUTY_CYCLE,
    };
    i2cStart(&I2CD1, &i2c_config);

    i2cAcquireBus(&I2CD1);
    uint8_t reg;

    // internal 25MHz oscillator, auto increment
    reg = PCA9685_MODE1_AI;
    pca9685_write_reg(PCA9685_MODE1, reg);

    // push-pull output, update on I2C STOP,
    reg = PCA9685_MODE2_OUTDRV;
    pca9685_write_reg(PCA9685_MODE2, reg);

    reg = PWM_PRESCALER - 1;
    pca9685_write_reg(PCA9685_PRE_SCALE, reg);

    // all servo output off
    unsigned int i;
    for (i = 0; i < SERVO_COUNT; i++) {
        uint8_t buf[5] = {
            PCA9685_LED0_ON_L + 4 * i,
            0, 0,       // LEDn_ON = 0
            0, 1 << 4   // LEDn_OFF[12] = 1 -> LEDn full OFF
        };
        i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, buf, sizeof(buf), NULL, 0);
    }
    i2cReleaseBus(&I2CD1);

    // output enable;
    palClearPad(GPIOC, GPIOC_SERVO_OUTPUT_EN);
}
