#include <string.h>
#include "uwb_protocol.h"

#define MAC_802_15_4_FC_DATA_FRAME      (0x1 << 0)
#define MAC_802_15_4_FC_PAN_ID_COMPRESS (0x1 << 6)
#define MAC_802_15_4_FC_SHORT_SRC_ADDR  (0x2 << 14)
#define MAC_802_15_4_FC_SHORT_DST_ADDR  (0x2 << 10)

#define MAC_HDR_LEN 9

void uwb_mac_encapsulate_frame(uint16_t pan_id,
                               uint16_t src_addr,
                               uint16_t dst_addr,
                               uint8_t *frame,
                               size_t frame_size)
{
    uint16_t frame_hdr = 0;
    frame_hdr |= MAC_802_15_4_FC_DATA_FRAME;
    frame_hdr |= MAC_802_15_4_FC_PAN_ID_COMPRESS;
    frame_hdr |= MAC_802_15_4_FC_SHORT_SRC_ADDR;
    frame_hdr |= MAC_802_15_4_FC_SHORT_DST_ADDR;

    // First, make room for header
    memmove(frame + MAC_HDR_LEN, frame, frame_size - MAC_HDR_LEN);

    // Then write frame control header
    frame[0] = frame_hdr & 0xff;
    frame[1] = frame_hdr >> 8;
    //frame[2]  is sequence number, unused for now
    frame[3] = pan_id & 0xff;
    frame[4] = pan_id >> 8;
    frame[5] = dst_addr & 0xff;
    frame[6] = dst_addr >> 8;
    frame[7] = src_addr & 0xff;
    frame[8] = src_addr >> 8;
}


void uwb_mac_decapsulate_frame(uint16_t *pan_id, uint16_t *src_addr, uint16_t *dst_addr, uint8_t *frame, size_t frame_size)
{
    *pan_id = (frame[4] << 8) | frame[3];
    *dst_addr = (frame[6] << 8) | frame[5];
    *src_addr = (frame[8] << 8) | frame[7];

    /* Discards header */
    memmove(frame, frame + MAC_HDR_LEN, frame_size - MAC_HDR_LEN);
}
