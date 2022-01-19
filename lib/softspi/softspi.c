#include <string.h>
#include "softspi/softspi.h"

void softspi_send(softspi_t* dev, const uint8_t* txbuf, uint8_t* rxbuf, size_t size)
{
    int curbyte;
    int curbit;
    int curbitpos;
    volatile int delay;

    memset(rxbuf, 0, size);

    for (size_t i = 0; i < 8 * size; i++) {
        curbyte = txbuf[i / 8];

        /* We transmit each byte MSB first. */
        curbitpos = 7 - (i % 8);
        curbit = (curbyte & (1 << curbitpos)) != 0;

        /* Set the current device input */
        softspi_mosi_set(dev, curbit);

        /* wait enough time after setting the data. */
        delay = dev->delay;
        while (delay--) {
        }

        softspi_sck_set(dev, 1);

        /* Wait enough time before reading the data. */
        delay = dev->delay;
        while (delay--) {
        }

        /* Read the current device output and store it in RX buf */
        curbit = softspi_miso_get(dev);
        rxbuf[i / 8] |= curbit << curbitpos;

        softspi_sck_set(dev, 0);
    }
}

void softspi_init(softspi_t* dev)
{
    softspi_sck_set(dev, 0);
}
