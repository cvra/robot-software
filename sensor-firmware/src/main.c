#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "vl6180x/vl6180x.h"
#include "bootloader_config.h"
#include "error/error.h"
#include "debug.h"
#include "main.h"

void i2c_init(void)
{
    /* I2C2 SCL */
    palSetPadMode(GPIOA, 9, PAL_STM32_OSPEED_HIGHEST |
                  PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_PUPDR_FLOATING |
                  PAL_MODE_ALTERNATE(4));
    /* I2C2 SDA */
    palSetPadMode(GPIOA, 10, PAL_STM32_OSPEED_HIGHEST |
                  PAL_STM32_OTYPE_OPENDRAIN |
                  PAL_STM32_PUPDR_FLOATING |
                  PAL_MODE_ALTERNATE(4));

    /* I2C1 setup */
    /* Configure clock to 100kHz */
    static const I2CConfig i2c_config = {
        // STM32_TIMINGR_PRESC(16U) |
        STM32_TIMINGR_PRESC(8U) |
        STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
        STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(19U),
        0,
        0,
    };
    i2cStart(&I2CD2, &i2c_config);
}

#define VL6180X_ADDRESS_0   0x29
#define VL6180X_ADDRESS_MOD 0x2A

#define VL6180X_GPIO0 PAL_LINE(GPIOA, 2U)
#define VL6180X_GPIO1 PAL_LINE(GPIOA, 3U)

vl6180x_t vl6180x_dev;

void tof_distance_init(void)
{
    /* GPIO setup */
    /* VL6180X GPIO1/INT */
    palSetLineMode(VL6180X_GPIO1, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);
    /* VL6180X GPIO0 */
    palSetLineMode(VL6180X_GPIO0, PAL_STM32_MODE_OUTPUT);

    /* VL6180X reset cycle */
    palClearLine(VL6180X_GPIO0);
    chThdSleepMilliseconds(1);
    palSetLine(VL6180X_GPIO0);
    chThdSleepMilliseconds(1);

    /* VL6180X TOF distance sensor setup */
    vl6180x_init(&vl6180x_dev, &I2CD2, VL6180X_ADDRESS_0);

    vl6180x_change_i2c_address(&vl6180x_dev, VL6180X_ADDRESS_MOD);

    if (vl6180x_ping(&vl6180x_dev)) {
        NOTICE("VL6180X ping...OK");
    } else {
        NOTICE("VL6180X ping...ERROR");
    }

    vl6180x_configure(&vl6180x_dev);
    NOTICE("VL6180X config");
}


#define TCS3472_I2C_ADDR        (0x29)
#define TCS3472_CMD             (1 << 7)
#define TCS3472_CMD_AUTO_INC    (1 << 5) | TCS3472_CMD
#define TCS3472_CMD_SPECIAL_FN  (3 << 5) | TCS3472_CMD

#define TCS3472_REG_ENABLE      (0x00)
#define TCS3472_REG_ATIME       (0x01)
#define TCS3472_REG_WTIME       (0x03)
#define TCS3472_REG_AILTL       (0x04)
#define TCS3472_REG_AILTH       (0x05)
#define TCS3472_REG_AIHTL       (0x06)
#define TCS3472_REG_AIHTH       (0x07)
#define TCS3472_REG_PERS        (0x0C)
#define TCS3472_REG_CONFIG      (0x0D)
#define TCS3472_REG_CONTROL     (0x0F)
#define TCS3472_REG_ID          (0x12) // 0x44 for TCS34725
#define TCS3472_REG_STATUS      (0x13)
#define TCS3472_REG_CDATAL      (0x14)
#define TCS3472_REG_CDATAH      (0x15)
#define TCS3472_REG_RDATAL      (0x16)
#define TCS3472_REG_RDATAH      (0x17)
#define TCS3472_REG_GDATAL      (0x18)
#define TCS3472_REG_GDATAH      (0x19)
#define TCS3472_REG_BDATAL      (0x1A)
#define TCS3472_REG_BDATAH      (0x1B)

void TCS3472_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t txbuf[2];
    txbuf[0] = reg | TCS3472_CMD_AUTO_INC;
    txbuf[1] = val;
    i2cMasterTransmitTimeout(&I2CD2, TCS3472_I2C_ADDR, txbuf, 2, NULL, 0, TIME_INFINITE);
}

uint8_t TCS3472_read_reg(uint8_t reg)
{
    uint8_t txbuf[1] = {reg | TCS3472_CMD_AUTO_INC};
    uint8_t rxbuf[1];
    i2cMasterTransmitTimeout(&I2CD2, TCS3472_I2C_ADDR, txbuf, 1, rxbuf, 1, TIME_INFINITE);
    return rxbuf[0];
}

void TCS3472_read_color(uint16_t crgb[4])
{
    while ((TCS3472_read_reg(TCS3472_REG_STATUS) & 1) == 0) {
        chThdSleepMilliseconds(1);
    }

    uint8_t txbuf[1] = {TCS3472_REG_CDATAL | TCS3472_CMD_AUTO_INC};
    uint8_t rxbuf[8];

    i2cMasterTransmitTimeout(&I2CD2, TCS3472_I2C_ADDR, txbuf, 1, rxbuf, 8, TIME_INFINITE);

    crgb[0] = rxbuf[0] | ((uint16_t)rxbuf[1]<<8);
    crgb[1] = rxbuf[2] | ((uint16_t)rxbuf[3]<<8);
    crgb[2] = rxbuf[4] | ((uint16_t)rxbuf[5]<<8);
    crgb[3] = rxbuf[6] | ((uint16_t)rxbuf[7]<<8);
}

void color_sensor_init(void)
{
    uint8_t id = TCS3472_read_reg(TCS3472_REG_ID);
    NOTICE("TCS3472_REG_ID = 0x%02x", id);

    TCS3472_write_reg(TCS3472_REG_ENABLE, 0b00001011); // PON, AEN, WEN -> enable continuous RGBC conversion, with wait
    TCS3472_write_reg(TCS3472_REG_ATIME, 0xF6); // 24 ms integration time
    TCS3472_write_reg(TCS3472_REG_WTIME, 0xAB); // 204 ms wait time
    TCS3472_write_reg(TCS3472_REG_CONFIG, 0x00); // normal wait time
    TCS3472_write_reg(TCS3472_REG_CONTROL, 0b00000010); // 16x gain
    NOTICE("TCS3472 init", id);
}

THD_FUNCTION(blinker, arg)
{
    (void) arg;
    while (1) {
        palSetPad(GPIOA, GPIOA_PIN0);
        chThdSleepMilliseconds(100);
        palClearPad(GPIOA, GPIOA_PIN0);
        chThdSleepMilliseconds(100);
    }
}

static void blinker_start(void)
{
    static THD_WORKING_AREA(blinker_wa, 256);
    chThdCreateStatic(blinker_wa, sizeof(blinker_wa), LOWPRIO, blinker, NULL);
}

void _unhandled_exception(void)
{
    chSysHalt("unhandled exception");

    while (true) {
        /* wait forever */
    }
}

bootloader_config_t config;

int main(void)
{
    halInit();
    chSysInit();

    debug_init();
    NOTICE("boot");

    blinker_start();

    if (!config_get(&config)) {
        chSysHalt("Cannot load config");
    }

    NOTICE("Board name=\"%s\", ID=%d", config.board_name, config.ID);

    i2c_init();
    NOTICE("I2C init");

    tof_distance_init();

    // Never returns
    uavcan_start(config.ID, config.board_name);

    return 0;
}
