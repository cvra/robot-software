#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include "pca9685_pwm.h"

#define PCA9685_25MHZ_CLOCK     25000000 // Hz
#define PCA9685_PRESCALER_MAX   256
#define PWM_COUNT_MAX           4095
#define PCA9685_NB_PWM          16

#define PCA9685_I2C_ADDR        0b01000000

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_LED0_ON         0x06
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_LED0_OFF        0x08
#define PCA9685_LED0_OFF_L      0x08
#define PCA9685_PRE_SCALE       0xFE

#define PCA9685_MODE1_SLEEP     (1 << 4)
#define PCA9685_MODE1_AI        (1 << 5)
#define PCA9685_MODE2_OUTDRV    (1 << 2)
#define PCA9685_MODE2_OCH       (1 << 3)

static float pca9685_pwm_period = 0.0;

static void pca9685_write_reg8(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, buf, 2, NULL, 0);
    i2cReleaseBus(&I2CD1);
}

static void pca9685_write_reg16(uint8_t reg_addr, uint16_t data)
{
    uint8_t buf[3] = {reg_addr, (uint8_t)data, (uint8_t)(data >> 8)};
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, buf, 3, NULL, 0);
    i2cReleaseBus(&I2CD1);
}

static uint8_t pca9685_read_reg8(uint8_t reg_addr)
{
    uint8_t data;
    i2cAcquireBus(&I2CD1);
    i2cMasterTransmit(&I2CD1, PCA9685_I2C_ADDR, &reg_addr, 1, &data, 1);
    i2cReleaseBus(&I2CD1);
    return data;
}

static void pwm_off(unsigned int pwm_nb)
{
    // LEDn_ON = 0
    pca9685_write_reg16(PCA9685_LED0_ON + 4 * pwm_nb, 0);

    // LEDn_OFF[12] = 1 -> LEDn full OFF
    pca9685_write_reg16(PCA9685_LED0_OFF + 4 * pwm_nb, (1 << 12));
}

static float range_limit(float min, float max, float val)
{
    if (val > max) {
        val = max;
    } else if (val < min) {
        val = min;
    }
    return val;
}

void pca9685_pwm_set_pulse_width(unsigned int pwm_nb, float pulse_width_sec)
{
    pca9685_pwm_set_duty_cycle(pwm_nb, pulse_width_sec / pca9685_pwm_period);
}

void pca9685_pwm_set_duty_cycle(unsigned int pwm_nb, float duty_cycle)
{
    if (pwm_nb >= PCA9685_NB_PWM) {
        WARNING("PWM %u does not exist", pwm_nb);
        return;
    }

    duty_cycle = range_limit(0.0, 1.0, duty_cycle);

    uint16_t count = (uint16_t)(PWM_COUNT_MAX * duty_cycle);

    pca9685_write_reg16(PCA9685_LED0_OFF + 4 * pwm_nb, count);
}

void pca9685_pwm_output_enable(bool enable)
{
    if (enable) {
        palClearPad(GPIOC, GPIOC_SERVO_OUTPUT_EN);
    } else {
        palSetPad(GPIOC, GPIOC_SERVO_OUTPUT_EN);
    }
}

static void pca9685_ll_init(void)
{
    palSetPadMode(GPIOB, GPIOB_I2C1_SCL, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetPadMode(GPIOB, GPIOB_I2C1_SDA, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    // output disable
    palSetPadMode(GPIOC, GPIOC_SERVO_OUTPUT_EN, PAL_MODE_OUTPUT_PUSHPULL);

    static const I2CConfig i2c_config = {
        OPMODE_I2C,
        100000,
        STD_DUTY_CYCLE,
    };
    i2cStart(&I2CD1, &i2c_config);
}

void pca9685_pwm_init(float period_sec)
{
    // configure prescaler (must be in sleep mode)
    pca9685_pwm_period = period_sec;
    float freq = PWM_COUNT_MAX / period_sec;
    uint32_t prescaler = (uint32_t)(PCA9685_25MHZ_CLOCK / freq);

    if (prescaler > PCA9685_PRESCALER_MAX || prescaler == 0) {
        ERROR("Impossible PWM frequency");
    } else if (PCA9685_25MHZ_CLOCK / prescaler != freq) {
        WARNING("Rounded PWM frequency: %f Hz -> %u Hz", freq, PCA9685_25MHZ_CLOCK / prescaler);
    }

    pca9685_ll_init();
    pca9685_pwm_output_enable(false);

    // internal 25MHz oscillator, auto increment, enter sleep mode
    pca9685_write_reg8(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_SLEEP);

    // change clock prescaler
    pca9685_write_reg8(PCA9685_PRE_SCALE, prescaler - 1);

    // reset sleep mode
    pca9685_write_reg8(PCA9685_MODE1, PCA9685_MODE1_AI);

    chThdSleepMilliseconds(1); // wait > 0.5us for oscillator

    // start PWM by clearing RESTART bit
    uint8_t reg;
    reg = pca9685_read_reg8(PCA9685_MODE1);
    pca9685_write_reg8(PCA9685_MODE1, reg);

    // push-pull output, update on I2C STOP,
    pca9685_write_reg8(PCA9685_MODE2, PCA9685_MODE2_OUTDRV);

    // all servo output off
    unsigned int i;
    for (i = 0; i < PCA9685_NB_PWM; i++) {
        pwm_off(i);
    }

    pca9685_pwm_output_enable(true);
}
