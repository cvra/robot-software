#include "serial_datagram.h"

#define END         '\xC0'
#define ESC         '\xDB'
#define ESC_END     '\xDC'
#define ESC_ESC     '\xDD'



uint32_t compute_crc(const char *buf, size_t len)
{
    return 0;
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
    char *crc_byte = (char *)&crc;
    int j;
    for (j = 0; j < 4; j++) {
        if (crc_byte[j] == ESC) {
            crc_and_end[i++] = ESC;
            crc_and_end[i++] = ESC_ESC;
        } else if (crc_byte[j] == END) {
            crc_and_end[i++] = END;
            crc_and_end[i++] = ESC_END;
        } else {
            crc_and_end[i++] = crc_byte[j];
        }
    }
    crc_and_end[i++] = END;
    send_fn(crc_and_end, i);
}

