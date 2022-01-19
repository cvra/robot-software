#include "stdint.h"
#include "pressure_sensor_interface.h"
#include <ch.h>
#include <hal.h>

#include "error/error.h"
#include "softspi/softspi.h"

softspi_t spi;

void mpr_start(void)
{
    softspi_init(&spi);
}

void softspi_sck_set(softspi_t* dev, int status)
{
    (void)dev;
    palWritePad(GPIOB, GPIOB_SPI_SCK, status);
}

void softspi_mosi_set(softspi_t* dev, int data)
{
    (void)dev;
    palWritePad(GPIOB, GPIOB_SPI_MOSI, data);
}

int softspi_miso_get(softspi_t* dev)
{
    (void)dev;
    return palReadPad(GPIOB, GPIOB_SPI_MISO);
}

static void mpr_select(void* arg)
{
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
}

static void mpr_transmit(void* arg, const uint8_t* tx, uint8_t* rx, size_t n)
{
    (void)arg;
    // The servo PWM causes issues with the SPI driver. Using software SPI as workaround.
    softspi_send(&spi, tx, rx, n);
}

mpr_driver_t pressure_sensors[2] = {{.arg = (void*)0,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect},
                                    {.arg = (void*)1,
                                     .transmit = mpr_transmit,
                                     .select = mpr_select,
                                     .unselect = mpr_unselect}};
