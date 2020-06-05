#include <ch.h>
#include <hal.h>
#include <uavcan/node.h>
#include "vl6180x/vl6180x.h"
#include "TCS3472.h"
#include "bootloader_config.h"
#include <error/error.h>
#include "debug.h"
#include "main.h"

void i2c_init(void)
{
    /* I2C2 SCL */
    palSetPadMode(GPIOA, 9, PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_FLOATING | PAL_MODE_ALTERNATE(4));
    /* I2C2 SDA */
    palSetPadMode(GPIOA, 10, PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN | PAL_STM32_PUPDR_FLOATING | PAL_MODE_ALTERNATE(4));

    /* I2C1 setup */
    /* Configure clock to 100kHz */
    static const I2CConfig i2c_config = {
        // STM32_TIMINGR_PRESC(16U) |
        STM32_TIMINGR_PRESC(8U) | STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) | STM32_TIMINGR_SCLH(15U) | STM32_TIMINGR_SCLL(19U),
        0,
        0,
    };
    i2cStart(&I2CD2, &i2c_config);
}

#define VL6180X_ADDRESS_0 0x29
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

TCS3472_t color_sensor;

void color_sensor_init(void)
{
    palSetLineMode(LED_WHITE, PAL_STM32_MODE_OUTPUT);
    palSetLine(LED_WHITE); // enable light

    TCS3472_init(&color_sensor, &I2CD2);

    if (TCS3472_ping(&color_sensor)) {
        NOTICE("TCS3472 ping OK");
    } else {
        NOTICE("TCS3472 ping ERROR");
    }

    TCS3472_configure(&color_sensor, TCS34721_GAIN_16X, TCS3472_INTEGRATION_TIME_101_MS);
}

THD_FUNCTION(blinker, arg)
{
    (void)arg;
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
#if USE_COLOR_SENSOR
    color_sensor_init();
#endif
    // Never returns
    uavcan_start(config.ID, config.board_name);

    return 0;
}
