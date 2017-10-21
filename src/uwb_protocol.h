#ifndef UWB_PROTOCOL_H
#define UWB_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <unistd.h>

/** Broadcast network ID. */
#define MAC_802_15_4_PAN_ID_BROADCAST 0xffff

/** Encapsulate frame data into a 802.15.4 MAC data frame.
 *
 * @param [in] pan_id Network ID to use (destination and source).
 * @param [in] src_addr The source address for this frame.
 * @param [in] dst_addr The destination address for this frame.
 * @param [in/out] frame. Frame data. It will be modified in place to add MAC
 * headers.
 */
void uwb_mac_encapsulate_frame(uint16_t pan_id,
                               uint16_t src_addr,
                               uint16_t dst_addr,
                               uint8_t *frame,
                               size_t frame_size);

/** Extracts frame data from a 802.15.4 MAC data frame.
 *
 * @param [out] pan_id Decoded PAN ID destination.
 * @param [out] src_addr Decoded Source address destination.
 * @param [out] dst_addr Decoded destination address destination.
 * @param [in/out] frame Frame data, will be modified in place to remove MAC headers.
*/
void uwb_mac_decapsulate_frame(uint16_t *pan_id, uint16_t *src_addr, uint16_t *dst_addr, uint8_t *frame, size_t frame_size);


#ifdef __cplusplus
}
#endif

#endif
