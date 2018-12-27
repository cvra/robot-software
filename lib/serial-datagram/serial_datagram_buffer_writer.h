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

void serial_datagram_buffer_writer_cb(void *arg, const void *buffer, size_t len);

/** @brief Wraps a buffer in a datagram.
 *
 * @return Length of the datagram
 */
size_t serial_datagram_buffer_wrap(const uint8_t *input_buffer, size_t input_buffer_len,
                                   uint8_t *output_buffer, size_t output_buffer_len);

#ifdef __cplusplus
}
#endif
#endif
