#ifndef HAL_H
#define HAL_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdlib.h>

/* Dummy functions implementing ChibiOS's HAL */

/* Generic */
void chThdSleepMilliseconds(int amount);

/* SPI */
typedef int SPIDriver;
void spiSelect(SPIDriver *p);
void spiSend(SPIDriver *p, size_t n, const void *buf);
void spiReceive(SPIDriver *spip, size_t n, void *rxbuf);
void spiUnselect(SPIDriver *spip);


#ifdef __cplusplus
}
#endif
#endif
