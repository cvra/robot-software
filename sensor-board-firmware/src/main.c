#include <ch.h>
#include <hal.h>
#include "uavcan/node.h"
#include "vl6180x/vl6180x.h"
#include "bootloader_config.h"
#include "error/error.h"
#include "debug.h"
#include "main.h"

#define VL6180X_ADDRESS 0x29

#define VL6180X_GPIO0 PAL_LINE(GPIOA, 2U)
#define VL6180X_GPIO1 PAL_LINE(GPIOA, 3U)

vl6180x_t tof_distance;

/** Initialize front front distance sensor */
void tof_distance_init(void)
{
    /* GPIO setup */

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
    /* VL6180X GPIO1/INT */
    palSetLineMode(VL6180X_GPIO1, PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_PULLUP);
    /* VL6180X GPIO0 */
    palSetLineMode(VL6180X_GPIO0, PAL_STM32_MODE_OUTPUT);

    /* VL6180X reset cycle */
    palClearLine(VL6180X_GPIO0);
    chThdSleepMilliseconds(1);
    palSetLine(VL6180X_GPIO0);
    chThdSleepMilliseconds(1);

    NOTICE("VL6180X GPIO init");

    /* I2C1 setup */
    /* Configure clock to 100kHz */
    static const I2CConfig i2c_config = {
        STM32_TIMINGR_PRESC(8) |
        STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
        STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(19U),
        0,
        0,
    };
    i2cStart(&I2CD2, &i2c_config);

    NOTICE("I2CD2 start");

    /* VL6180X TOF distance sensor setup */
    vl6180x_init(&tof_distance, &I2CD2, VL6180X_ADDRESS);
    vl6180x_configure(&tof_distance);
    NOTICE("VL6180X config");
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

    tof_distance_init();
    while (1) {
        uint8_t distance_mm, status;
        status = vl6180x_measure_distance(&tof_distance, &distance_mm);
        NOTICE("distance [mm]: %d (status = %02x)", distance_mm, status);
        chThdSleepMilliseconds(100);
    }

    // Never returns
    // uavcan_start(10, "hello");

    return 0;
}
