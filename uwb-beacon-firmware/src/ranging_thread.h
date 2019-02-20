#ifndef RANGING_THREAD_H
#define RANGING_THREAD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint32_t timestamp; ///< Time at which the ranging solution was found (in us since boot)
    uint16_t anchor_addr; ///< Address of the anchor with which the measurement was done
    float range; ///< Distance to the anchor, in meters
} range_msg_t;

typedef struct {
    uint32_t timestamp;
    uint16_t anchor_addr;
    float x;
    float y;
    float z;
} anchor_position_msg_t;

typedef struct {
    uint32_t timestamp;
    uint16_t tag_addr;
    float x;
    float y;
} tag_position_msg_t;

/** Struct used on the data packet topic */
typedef struct {
    uint16_t src_mac;
    uint16_t dst_mac;
    size_t data_size;
    uint8_t data[1024];
} data_packet_msg_t;

void ranging_start(void);

/** Asks the ranging thread to send this data packet when possible.
 *
 * Blocks until the packet is sent to the DWM1000 (returns before the packet is
 * actually transmitted).
 */
void ranging_send_data_packet(const uint8_t* data, size_t data_size, uint16_t dst_mac);

#ifdef __cplusplus
}
#endif

#endif
