#include <string.h>
#include "uwb_protocol.h"

#define MAC_802_15_4_FC_DATA_FRAME      (0x1 << 0)
#define MAC_802_15_4_FC_PAN_ID_COMPRESS (0x1 << 6)
#define MAC_802_15_4_FC_SHORT_SRC_ADDR  (0x2 << 14)
#define MAC_802_15_4_FC_SHORT_DST_ADDR  (0x2 << 10)

#define MAC_HDR_LEN                     9
#define MAC_CRC_LEN                     2

static void write_40bit_int(uint64_t val, uint8_t *bytes);

size_t uwb_mac_encapsulate_frame(uint16_t pan_id,
                                 uint16_t src_addr,
                                 uint16_t dst_addr,
                                 uint8_t seq_number,
                                 uint8_t *frame,
                                 size_t frame_size)
{
    uint16_t frame_hdr = 0;
    frame_hdr |= MAC_802_15_4_FC_DATA_FRAME;
    frame_hdr |= MAC_802_15_4_FC_PAN_ID_COMPRESS;
    frame_hdr |= MAC_802_15_4_FC_SHORT_SRC_ADDR;
    frame_hdr |= MAC_802_15_4_FC_SHORT_DST_ADDR;

    /* Make room for header */
    memmove(frame + MAC_HDR_LEN, frame, frame_size);

    /* Write frame control header */
    frame[0] = frame_hdr & 0xff;
    frame[1] = frame_hdr >> 8;
    frame[2] = seq_number;
    frame[3] = pan_id & 0xff;
    frame[4] = pan_id >> 8;
    frame[5] = dst_addr & 0xff;
    frame[6] = dst_addr >> 8;
    frame[7] = src_addr & 0xff;
    frame[8] = src_addr >> 8;
    frame_size += MAC_HDR_LEN;

    /* Clears CRC placeholders */
    for (int i = 0; i < MAC_CRC_LEN; i++) {
        frame[frame_size + i] = 0x00;
    }

    frame_size += MAC_CRC_LEN;

    return frame_size;
}


size_t uwb_mac_decapsulate_frame(uint16_t *pan_id,
                                 uint16_t *src_addr,
                                 uint16_t *dst_addr,
                                 uint8_t *seq,
                                 uint8_t *frame,
                                 size_t frame_size)
{
    *seq = frame[2];
    *pan_id = (frame[4] << 8) | frame[3];
    *dst_addr = (frame[6] << 8) | frame[5];
    *src_addr = (frame[8] << 8) | frame[7];

    /* Discards header */
    memmove(frame, frame + MAC_HDR_LEN, frame_size - MAC_HDR_LEN);

    return frame_size - MAC_HDR_LEN - MAC_CRC_LEN;
}

static void write_40bit_int(uint64_t val, uint8_t *bytes)
{
    bytes[0] = (val >> 32) & 0xff;
    bytes[1] = (val >> 24) & 0xff;
    bytes[2] = (val >> 16) & 0xff;
    bytes[3] = (val >> 8) & 0xff;
    bytes[4] = (val >> 0) & 0xff;
}

void uwb_protocol_handler_init(uwb_protocol_handler_t *handler)
{
    memset(handler, 0, sizeof(uwb_protocol_handler_t));
}

size_t uwb_protocol_prepare_measurement_advertisement(uwb_protocol_handler_t *handler,
                                                      uint64_t tx_timestamp,
                                                      uint8_t *frame)
{
    size_t size;
    write_40bit_int(tx_timestamp, frame);
    size = uwb_mac_encapsulate_frame(handler->pan_id,
                                     handler->address,
                                     MAC_802_15_4_BROADCAST_ADDR,
                                     0,
                                     frame,
                                     5);

    return size;
}

void uwb_send_measurement_advertisement(uwb_protocol_handler_t *handler, uint8_t *buffer)
{
    uint64_t ts = uwb_timestamp_get();
    size_t frame_size;

    // TODO: Is this the correct place to add some delay?
    ts += 1000;

    frame_size = uwb_protocol_prepare_measurement_advertisement(handler, ts, buffer);
    uwb_transmit_frame(ts, buffer, frame_size);
}

void uwb_process_incoming_frame(uwb_protocol_handler_t *handler,
                                uint8_t *frame,
                                size_t frame_size,
                                uint64_t rx_ts)
{
    uint16_t pan_id;
    uint16_t src_addr;
    uint16_t dst_addr;
    uint8_t seq_num;
    frame_size = uwb_mac_decapsulate_frame(&pan_id,
                                           &src_addr,
                                           &dst_addr,
                                           &seq_num,
                                           frame,
                                           frame_size);

    /* Checks that the packet comes from the correct PAN. */
    if (pan_id != handler->pan_id) {
        return;
    }

    /* Checks that the packet is sent to us. The hardware should already do
     * this filter, but checking protects us against misconfigurations. */
    if (dst_addr != handler->address && dst_addr != MAC_802_15_4_BROADCAST_ADDR) {
        return;
    }

    /* Measurement advertisement */
    if (seq_num == 0) {
        // TODO how to properly handle this delay
        uint64_t reply_ts = rx_ts + 1000;

        /* Do not change the advertisement TX timestamp, append the reply RX &
         * TX timestamps. */
        write_40bit_int(rx_ts, &frame[5]);
        write_40bit_int(reply_ts, &frame[10]);
        frame_size += 10;

        /* Encodes an answer back to the source. */
        dst_addr = src_addr;
        src_addr = handler->address;
        frame_size = uwb_mac_encapsulate_frame(pan_id,
                                               src_addr,
                                               dst_addr,
                                               seq_num + 1,
                                               frame,
                                               frame_size);
        /* Sends the answer. */
        uwb_transmit_frame(reply_ts, frame, frame_size);
    } else if (seq_num == 1) {
        // TODO how to properly handle this delay
        uint64_t reply_ts = rx_ts + 1000;

        /* Do not change the advertisement & reply TX timestamp, append the
         * reply RX & final TX timestamps. */
        write_40bit_int(rx_ts, &frame[15]);
        write_40bit_int(reply_ts, &frame[20]);
        frame_size += 10;

        /* Encodes an answer back to the source. */
        dst_addr = src_addr;
        src_addr = handler->address;
        frame_size = uwb_mac_encapsulate_frame(pan_id,
                                               src_addr,
                                               dst_addr,
                                               seq_num + 1,
                                               frame,
                                               frame_size);
        /* Sends the answer. */
        uwb_transmit_frame(reply_ts, frame, frame_size);
    }
}
