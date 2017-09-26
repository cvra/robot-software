#include <stdint.h>
#include <stdbool.h>
#include <crc/crc32.h>
#include "serial_datagram.h"

#define END         (uint8_t)'\xC0'
#define ESC         (uint8_t)'\xDB'
#define ESC_END     (uint8_t)'\xDC'
#define ESC_ESC     (uint8_t)'\xDD'


static uint32_t compute_crc(const void *buf, size_t len)
{
    return crc32(SERIAL_DATAGRAM_CRC_START, buf, len);
}

static const uint8_t esc_end[] = {ESC, ESC_END};
static const uint8_t esc_esc[] = {ESC, ESC_ESC};

void serial_datagram_send_chunk(const void *dtgrm, size_t len, uint32_t *crc,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg)
{
    const uint8_t *dtgrm_byte = (const uint8_t*)dtgrm;
    // send escaped data
    uint32_t a = 0, b = 0;
    while (b < len) {
        if (dtgrm_byte[b] == END) {
            send_fn(sendarg, &dtgrm_byte[a], b - a);
            send_fn(sendarg, esc_end, 2);
            a = b + 1;
        } else if (dtgrm_byte[b] == ESC) {
            send_fn(sendarg, &dtgrm_byte[a], b - a);
            send_fn(sendarg, esc_esc, 2);
            a = b + 1;
        }
        b++;
    }
    send_fn(sendarg, &dtgrm_byte[a], b - a);

    // update CRC
    *crc = crc32(*crc, dtgrm, len);
}

void serial_datagram_send_end(uint32_t crc,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg)
{
    // send CRC32 + END
    uint8_t crc_and_end[2*4 + 1]; // escaped CRC32 + END
    uint32_t i = 0;
    int32_t j;
    for (j = 3*8; j >= 0; j -= 8) {
        if (((crc >> j) & 0xFF) == ESC) {
            crc_and_end[i++] = esc_esc[0];
            crc_and_end[i++] = esc_esc[1];
        } else if (((crc >> j) & 0xFF) == END) {
            crc_and_end[i++] = esc_end[0];
            crc_and_end[i++] = esc_end[1];
        } else {
            crc_and_end[i++] = ((crc >> j) & 0xFF);
        }
    }
    crc_and_end[i++] = END;
    send_fn(sendarg, crc_and_end, i);
}

void serial_datagram_send(const void *dtgrm, size_t len,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg)
{
    uint32_t crc = SERIAL_DATAGRAM_CRC_START;
    serial_datagram_send_chunk(dtgrm, len, &crc, send_fn, sendarg);
    serial_datagram_send_end(crc, send_fn, sendarg);
}

static void rcv_handler_reset(serial_datagram_rcv_handler_t *h)
{
    h->write_index = 0;
    h->error_flag = false;
    h->esc_flag = false;
}

void serial_datagram_rcv_handler_init(serial_datagram_rcv_handler_t *h,
        void *buffer, size_t size, serial_datagram_cb_t cb_fn, void *cb_arg)
{
    h->buffer = (uint8_t*)buffer;
    h->size = size;
    h->callback_fn = cb_fn;
    h->callback_arg = cb_arg;
    rcv_handler_reset(h);
}

int serial_datagram_receive(serial_datagram_rcv_handler_t *h, const void *in,
        size_t len)
{
    const uint8_t *read = (const uint8_t *)in;
    int error_code = SERIAL_DATAGRAM_RCV_NO_ERROR;
    while (len--) {
        if (h->error_flag) {
            if (*read == END) {
                rcv_handler_reset(h);
            }
        } else if (*read == END) {
            int datagram_len = h->write_index - 4;
            if (datagram_len >= 0) {
                uint32_t crc = compute_crc(h->buffer, datagram_len);
                uint32_t received_crc = (
                    ((uint32_t)h->buffer[h->write_index - 4] & 0xff) << 3*8 |
                    ((uint32_t)h->buffer[h->write_index - 3] & 0xff) << 2*8 |
                    ((uint32_t)h->buffer[h->write_index - 2] & 0xff) << 1*8 |
                    ((uint32_t)h->buffer[h->write_index - 1] & 0xff) );
                if (crc != received_crc) {
                    error_code = SERIAL_DATAGRAM_RCV_CRC_MISMATCH;
                } else {
                    h->callback_fn(h->buffer, datagram_len, h->callback_arg);
                }
            } else {
                error_code = SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR;
            }
            rcv_handler_reset(h);
        } else if (h->write_index == h->size) {
            h->error_flag = true;
            error_code = SERIAL_DATAGRAM_RCV_DATAGRAM_TOO_LONG;
        } else if (h->esc_flag) {
            if (*read == ESC_ESC) {
                h->buffer[h->write_index++] = ESC; // write data byte ESC
            } else if (*read == ESC_END) {
                h->buffer[h->write_index++] = END; // write data byte END
            } else { // invalid escape sequence
                error_code = SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR;
                h->error_flag = true;
            }
            h->esc_flag = false;
        } else if (*read == ESC) {
            h->esc_flag = true;
        } else {
             h->buffer[h->write_index++] = *read; // write data byte *read
        }
        read++;
    }
    return error_code;
}
