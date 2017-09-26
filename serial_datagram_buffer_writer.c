#include "serial_datagram.h"
#include "serial_datagram_buffer_writer.h"


void serial_datagram_buffer_writer_init(serial_datagram_buffer_writer_t *writer, uint8_t *buffer, size_t buffer_size)
{
    writer->buffer = buffer;
    writer->buffer_size = buffer_size;
    writer->write_index = 0;
}

void serial_datagram_buffer_writer_cb(void *arg, const void *buffer, size_t len)
{
    size_t i;
    serial_datagram_buffer_writer_t *writer = (serial_datagram_buffer_writer_t *)arg;
    uint8_t *data = (uint8_t *)buffer;

    for (i = 0; i < len; ++i) {
        if (writer->write_index < writer->buffer_size) {
            writer->buffer[writer->write_index] = data[i];
            writer->write_index ++;
        }
    }
}

size_t serial_datagram_buffer_wrap(const uint8_t *input_buffer, size_t input_buffer_len,
                                   uint8_t *output_buffer, size_t output_buffer_len)
{
    serial_datagram_buffer_writer_t writer;
    serial_datagram_buffer_writer_init(&writer, output_buffer, output_buffer_len);

    serial_datagram_send((void *)input_buffer, input_buffer_len,
                         serial_datagram_buffer_writer_cb, (void *)&writer);

    return writer.write_index;
}
