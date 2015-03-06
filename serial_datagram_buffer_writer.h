#ifndef SERIAL_DATAGRAM_BUFFER_WRITER_H
#define SERIAL_DATAGRAM_BUFFER_WRITER_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Instance of an in-memory writer for serial datagrams. */
typedef struct {
    uint8_t *buffer;
    size_t buffer_size;
    size_t write_index;
} serial_datagram_buffer_writer_t;

void serial_datagram_buffer_writer_init(serial_datagram_buffer_writer_t *writer, uint8_t *buffer, size_t buffer_size);

void serial_datagram_buffer_writer_cb(void *arg, void *buffer, size_t len);

#ifdef __cplusplus
}
#endif
#endif
