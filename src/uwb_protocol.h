#ifndef UWB_PROTOCOL_H
#define UWB_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <unistd.h>

/** Broadcast address. Also work as a PAN ID */
#define MAC_802_15_4_BROADCAST_ADDR 0xffff

/** Object handling all the UWB protocol interactions. */
typedef struct {
    uint16_t pan_id;
    uint16_t address;
    void (*ranging_found_cb)(uint16_t anchor_addr, uint64_t time);
} uwb_protocol_handler_t;

/** Encapsulate frame data into a 802.15.4 MAC data frame.
 *
 * @param [in] pan_id Network ID to use (destination and source).
 * @param [in] src_addr The source address for this frame.
 * @param [in] dst_addr The destination address for this frame.
 * @param [in] seq_number The sequence number of this frame
 * @param [in/out] frame. Frame data. It will be modified in place to add MAC
 * headers.
 *
 * @returns The size of the frame, including header & checksum placeolder.
 */
size_t uwb_mac_encapsulate_frame(uint16_t pan_id,
                                 uint16_t src_addr,
                                 uint16_t dst_addr,
                                 uint8_t seq_number,
                                 uint8_t *frame,
                                 size_t frame_size);

/** Extracts frame data from a 802.15.4 MAC data frame.
 *
 * @param [out] pan_id Decoded PAN ID destination.
 * @param [out] src_addr Decoded Source address destination.
 * @param [out] dst_addr Decoded destination address destination.
 * @param [out] seq The sequence number for this packet.
 * @param [in/out] frame Frame data, will be modified in place to remove MAC headers.
 *
 * @returns Payload size in bytes.
 */
size_t uwb_mac_decapsulate_frame(uint16_t *pan_id,
                                 uint16_t *src_addr,
                                 uint16_t *dst_addr,
                                 uint8_t *seq,
                                 uint8_t *frame,
                                 size_t frame_size);

void uwb_protocol_handler_init(uwb_protocol_handler_t *handler);

/** Prepares a measurement frame.
 *
 * @returns Number of byte in frame (including MAC header & 2 byte placeholder
 * for CRC).
 */
size_t uwb_protocol_prepare_measurement_advertisement(uwb_protocol_handler_t *handler,
                                                      uint64_t tx_timestamp,
                                                      uint8_t *frame);

void uwb_send_measurement_advertisement(uwb_protocol_handler_t *handler, uint8_t *buffer);

void uwb_process_incoming_frame(uwb_protocol_handler_t *handler, uint8_t *frame, size_t frame_size, uint64_t rx_ts);


/** @group UWB Board specific API
 *
 * @brief Those functions must be provided on a per-board basis to interface
 * with the UWB module. Typically, they would be either shims around the
 * decawave API or a custom implementation.
 * @{
 */

/** Send the given frame at the correct timestamp. */
extern void uwb_transmit_frame(uint64_t tx_timestamp, uint8_t *frame, size_t frame_size);

/** Reads the current value of the UWB module clock. */
extern uint64_t uwb_timestamp_get(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
