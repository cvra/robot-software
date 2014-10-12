#include <stdint.h>
#include <stdbool.h>
// #include <crc/crc32.h>
#include "serial_datagram.h"

#define END         '\xC0'
#define ESC         '\xDB'
#define ESC_END     '\xDC'
#define ESC_ESC     '\xDD'


uint32_t compute_crc(const char *buf, size_t len)
{
    return 0x11223344;
}

void serial_datagram_send(const char *dtgrm, size_t len,
        void (*send_fn)(const char *p, size_t len))
{
    const static char esc_end[] = {ESC, ESC_END};
    const static char esc_esc[] = {ESC, ESC_ESC};
    // send escaped data
    int a = 0, b = 0;
    while (b < len) {
        if (dtgrm[b] == END) {
            send_fn(&dtgrm[a], b - a);
            send_fn(esc_end, 2);
            a = b + 1;
        } else if (dtgrm[b] == ESC) {
            send_fn(&dtgrm[a], b - a);
            send_fn(esc_esc, 2);
            a = b + 1;
        }
        b++;
    }
    send_fn(&dtgrm[a], b - a);

    // send CRC32 + END
    char crc_and_end[2*4 + 1]; // escaped CRC32 + END
    int i = 0;
    uint32_t crc = compute_crc(dtgrm, len);
    int j;
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
    send_fn(crc_and_end, i);
}

static void rcv_handler_reset(serial_datagram_rcv_handler_t *h)
{
    h->write_index = 0;
    h->error_flag = false;
    h->esc_flag = false;
}

void serial_datagram_rcv_handler_init(serial_datagram_rcv_handler_t *h,
        char *buffer, size_t size, void (*cb)(const char *dtgrm, size_t len))
{
    h->buffer = buffer;
    h->size = size;
    h->callback_fn = cb;
    rcv_handler_reset(h);
}

int serial_datagram_receive(serial_datagram_rcv_handler_t *h, const char *in,
        size_t len)
{
    int error_code = SERIAL_DATAGRAM_RCV_NO_ERROR;
    while (len--) {
        if (h->error_flag) {
            if (*in == END) {
                rcv_handler_reset(h);
            }
        } else if (*in == END) {
            int datagram_len = h->write_index - 4;
            uint32_t crc = compute_crc(h->buffer, datagram_len);
            uint32_t received_crc = ( h->buffer[h->write_index - 4] << 3*8 |
                                      h->buffer[h->write_index - 3] << 2*8 |
                                      h->buffer[h->write_index - 2] << 1*8 |
                                      h->buffer[h->write_index - 1] );
            if (crc != received_crc) {
                error_code = SERIAL_DATAGRAM_RCV_CRC_MISMATCH;
            } else {
                h->callback_fn(h->buffer, datagram_len);
            }
            rcv_handler_reset(h);
        } else if (h->write_index == h->size) {
            h->error_flag = true;
            error_code = SERIAL_DATAGRAM_RCV_DATAGRAM_TOO_LONG;
        } else if (h->esc_flag) {
            if (*in == ESC_ESC) {
                h->buffer[h->write_index++] = ESC; // write data byte ESC
            } else if (*in == ESC_END) {
                h->buffer[h->write_index++] = END; // write data byte END
            } else { // invalid escape sequence
                error_code = SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR;
                h->error_flag = true;
            }
            h->esc_flag = false;
        } else if (*in == ESC) {
            h->esc_flag = true;
        } else {
             h->buffer[h->write_index++] = *in; // write data byte *in
        }
        in++;
    }
    return error_code;
}
