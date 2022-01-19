#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    /** Delay is simply a constant used to adjust the clock rate. It is the
     * number of time a busy wait loop will be executed, so it is not really
     * linked to anything physical. */
    int delay;
} softspi_t;

void softspi_init(softspi_t* dev);

/** Transfers bytes on SPI and waits for the transfer to finish, simulatenously reading data from the device.
 *
 * The only SPI mode that is supported is 8 bit word, clock polarity 0, clock phase 0.
 */
void softspi_send(softspi_t* dev, const uint8_t* txbuf, uint8_t* rxbuf, size_t size);

/* These functions must be provided to interface with the hardware. */
extern void softspi_sck_set(softspi_t* dev, int status);
extern void softspi_mosi_set(softspi_t* dev, int data);
extern int softspi_miso_get(softspi_t* dev);

#ifdef __cplusplus
}
#endif
