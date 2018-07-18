#include <string.h>
#include "uwb_protocol.h"

#define MAC_802_15_4_FC_DATA_FRAME      (0x1 << 0)
#define MAC_802_15_4_FC_PAN_ID_COMPRESS (0x1 << 6)
#define MAC_802_15_4_FC_SHORT_SRC_ADDR  (0x2 << 14)
#define MAC_802_15_4_FC_SHORT_DST_ADDR  (0x2 << 10)

#define MAC_HDR_LEN                     9
#define MAC_CRC_LEN                     2

#define UWB_SEQ_NUM_ADVERTISEMENT           0
#define UWB_SEQ_NUM_REPLY                   1
#define UWB_SEQ_NUM_FINALIZATION            2
#define UWB_SEQ_NUM_ANCHOR_POSITION         3
#define UWB_SEQ_NUM_TAG_POSITION            4
#define UWB_SEQ_NUM_INITIATE_MEASUREMENT    5

#define UWB_DELAY                       (1000 * 65536)
#define MASK_40BIT                      0xfffffffe00

static void write_40bit_int(uint64_t val, uint8_t *bytes);
static uint64_t read_40bit_int(uint8_t *bytes);

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

static uint64_t read_40bit_int(uint8_t *bytes)
{
    uint64_t res = 0;
    for (int i = 0; i < 5; i++) {
        res = (res << 8) | bytes[i];
    }
    return res;
}

/** Computes the difference between two unsigned 40 bit numbers, correctly
 * handling overflows. */
static uint64_t substract_40bit_int(uint64_t a, uint64_t b)
{
    const uint64_t uint40_max = (1ULL << 40) - 1;
    uint64_t res = a - b;
    if (b > a) {
        res += uint40_max;;
    }

    return res;
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
                                     UWB_SEQ_NUM_ADVERTISEMENT,
                                     frame,
                                     5);

    return size;
}

void uwb_send_measurement_advertisement(uwb_protocol_handler_t *handler, uint8_t *buffer)
{
    uint64_t ts = uwb_timestamp_get();
    size_t frame_size;

    // TODO: Is this the correct place to add some delay?
    ts += UWB_DELAY;
    ts &= MASK_40BIT;

    frame_size = uwb_protocol_prepare_measurement_advertisement(handler, ts, buffer);
    uwb_transmit_frame(ts, buffer, frame_size);
}

size_t uwb_protocol_prepare_anchor_position(uwb_protocol_handler_t *handler,
                                            float x,
                                            float y,
                                            float z,
                                            uint8_t *frame)
{
    /* TODO Use proper endianness */
    float *msg = (float *)frame;
    msg[0] = x;
    msg[1] = y;
    msg[2] = z;

    return uwb_mac_encapsulate_frame(handler->pan_id,
                                     handler->address,
                                     MAC_802_15_4_BROADCAST_ADDR,
                                     UWB_SEQ_NUM_ANCHOR_POSITION,
                                     frame,
                                     12);
}

size_t uwb_protocol_prepare_tag_position(uwb_protocol_handler_t *handler,
                                         float x,
                                         float y,
                                         uint8_t *frame)
{
    /* TODO Use proper endianness */
    float *msg = (float *)frame;
    msg[0] = x;
    msg[1] = y;

    return uwb_mac_encapsulate_frame(handler->pan_id,
                                     handler->address,
                                     MAC_802_15_4_BROADCAST_ADDR,
                                     UWB_SEQ_NUM_TAG_POSITION,
                                     frame,
                                     8);
}


void uwb_send_anchor_position(uwb_protocol_handler_t *handler,
                              float x,
                              float y,
                              float z,
                              uint8_t *frame)
{
    size_t size;

    size = uwb_protocol_prepare_anchor_position(handler, x, y, z, frame);

    uwb_transmit_frame(UWB_TX_TIMESTAMP_IMMEDIATE, frame, size);
}

void uwb_send_tag_position(uwb_protocol_handler_t *handler, float x, float y, uint8_t *buffer)
{
    size_t size;

    size = uwb_protocol_prepare_tag_position(handler, x, y, buffer);

    uwb_transmit_frame(UWB_TX_TIMESTAMP_IMMEDIATE, buffer, size);
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
    if (seq_num == UWB_SEQ_NUM_ADVERTISEMENT) {
        if (handler->is_anchor == false) {
            // TODO how to properly handle this delay
            uint64_t reply_ts = rx_ts + UWB_DELAY;

            reply_ts &= MASK_40BIT;

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
                                                   UWB_SEQ_NUM_REPLY,
                                                   frame,
                                                   frame_size);
            /* Sends the answer. */
            uwb_transmit_frame(reply_ts, frame, frame_size);
        }
    } else if (seq_num == UWB_SEQ_NUM_REPLY) {
        // TODO how to properly handle this delay
        uint64_t reply_ts = rx_ts + UWB_DELAY;

        reply_ts &= MASK_40BIT;

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
                                               UWB_SEQ_NUM_FINALIZATION,
                                               frame,
                                               frame_size);
        /* Sends the answer. */
        uwb_transmit_frame(reply_ts, frame, frame_size);

    } else if (seq_num == UWB_SEQ_NUM_FINALIZATION) {
        uint64_t advertisement_tx_ts = read_40bit_int(&frame[0]);
        uint64_t advertisement_rx_ts = read_40bit_int(&frame[5]);
        uint64_t reply_tx_ts = read_40bit_int(&frame[10]);
        uint64_t reply_rx_ts = read_40bit_int(&frame[15]);
        uint64_t final_tx_ts = read_40bit_int(&frame[20]);
        uint64_t final_rx_ts = rx_ts;
        uint64_t t_propag;
        uint64_t t_ranging;

        // See documentation for explanation
        uint64_t tround[2], treply[2];

        tround[0] = substract_40bit_int(reply_rx_ts, advertisement_tx_ts);
        tround[1] = substract_40bit_int(final_rx_ts, reply_tx_ts);
        treply[0] = substract_40bit_int(reply_tx_ts, advertisement_rx_ts);
        treply[1] = substract_40bit_int(final_tx_ts, reply_rx_ts);

        t_propag = (tround[0] * tround[1]  - treply[0] * treply[1]) /
                   (tround[0] + tround[1]  + treply[0] + treply[1]);

        t_ranging = substract_40bit_int(final_tx_ts, advertisement_rx_ts);

        if (handler->ranging_found_cb) {
            handler->ranging_found_cb(src_addr, t_propag, t_ranging);
        }
    } else if (seq_num == UWB_SEQ_NUM_ANCHOR_POSITION) {
        float x, y, z;
        memcpy(&x, &frame[0], sizeof(float));
        memcpy(&y, &frame[4], sizeof(float));
        memcpy(&z, &frame[8], sizeof(float));
        handler->anchor_position_received_cb(src_addr, x, y, z);
    } else if (seq_num == UWB_SEQ_NUM_TAG_POSITION) {
        float x, y;
        memcpy(&x, &frame[0], sizeof(float));
        memcpy(&y, &frame[4], sizeof(float));
        handler->tag_position_received_cb(src_addr, x, y);
    } else if (seq_num == UWB_SEQ_NUM_INITIATE_MEASUREMENT) {
        uwb_send_measurement_advertisement(handler, frame);
    }
}

void uwb_initiate_measurement(uwb_protocol_handler_t *handler, uint8_t *buffer, uint16_t anchor_addr)
{
    uint64_t ts = uwb_timestamp_get();

    size_t size;
    size = uwb_mac_encapsulate_frame(handler->pan_id,
                                     handler->address,
                                     anchor_addr,
                                     UWB_SEQ_NUM_INITIATE_MEASUREMENT,
                                     buffer,
                                     0);

    // TODO: Is this the correct place to add some delay?
    ts += UWB_DELAY;
    ts &= MASK_40BIT;

    uwb_transmit_frame(ts, buffer, size);
}
