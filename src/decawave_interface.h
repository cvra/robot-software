#ifndef DECAWAVE_INTERFACE_H
#define DECAWAVE_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint32_t timestamp;   ///< Time at which the ranging solution was found
    uint16_t anchor_addr; ///< Address of the anchor with which the measurement was done
    float range;          ///< Distance to the anchor, in meters
} range_msg_t;

/** Starts the communication interface to the chip. */
void decawave_start(void);

#ifdef __cplusplus
}
#endif

#endif
