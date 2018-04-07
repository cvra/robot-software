#ifndef DECAWAVE_INTERFACE_H
#define DECAWAVE_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Starts the communication interface to the chip. */
void decawave_start(void);
uint64_t decawave_get_rx_timestamp_u64(void);

#ifdef __cplusplus
}
#endif

#endif
