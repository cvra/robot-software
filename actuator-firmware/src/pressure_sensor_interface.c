#include "board.h"
#include "pressure_sensor_interface.h"
#include <ch.h>
#include <hal.h>

static void mpr_select(void* arg)
{
    // SPI3 clock: PCLK1 = 36MHz
    static SPIConfig config = {
        .circular = false,
        .end_cb = NULL,
        .ssport = GPIOA,
        .sspad = GPIOA_CS1,
        .cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0,
        .cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0};

    spiAcquireBus(&SPID3);
    if ((int)arg == 0) {
        config.sspad = GPIOA_CS1;
    } else {
        config.sspad = GPIOA_CS2;
    }
    spiStart(&SPID3, &config);
    spiSelect(&SPID3);
}

static void mpr_unselect(void* arg)
{
    (void)arg;
    spiUnselect(&SPID3);
    spiReleaseBus(&SPID3);
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
