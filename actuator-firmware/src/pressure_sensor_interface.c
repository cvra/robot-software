#include "pressure_sensor_interface.h"
#include <ch.h>
#include <hal.h>

#include "error/error.h"

void mpr_start(void)
{
    // SPI3 clock: PCLK1 = 36MHz
    static SPIConfig config = {
        .circular = false,
        .end_cb = NULL,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
        .cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0};

    spiStart(&SPID3, &config);
}

static void mpr_select(void* arg)
{
#if SPI_USE_MUTUAL_EXCLUSION == TRUE
    spiAcquireBus(&SPID3);
#endif

    if ((int)arg == 0) {
        palClearPad(GPIOA, GPIOA_CS1);
    } else {
        palClearPad(GPIOA, GPIOA_CS2);
    }
}

static void mpr_unselect(void* arg)
{
    (void)arg;
    if ((int)arg == 0) {
        palSetPad(GPIOA, GPIOA_CS1);
    } else {
        palSetPad(GPIOA, GPIOA_CS2);
    }

#if SPI_USE_MUTUAL_EXCLUSION == TRUE
    spiReleaseBus(&SPID3);
#endif
}

static void mpr_transmit(void* arg, const uint8_t* tx, uint8_t* rx, size_t n)
{
    (void)arg;
    spiExchange(&SPID3, n, tx, rx);
}

mpr_driver_t pressure_sensors[2] = {{.arg = (void*)0,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect},
                                    {.arg = (void*)1,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect}};
