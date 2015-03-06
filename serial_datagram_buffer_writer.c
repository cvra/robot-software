#include "serial_datagram_buffer_writer.h"


void serial_datagram_buffer_writer_init(serial_datagram_buffer_writer_t *writer, uint8_t *buffer, size_t buffer_size)
{
    writer->buffer = buffer;
    writer->buffer_size = buffer_size;
    writer->write_index = 0;
}

void serial_datagram_buffer_writer_cb(void *arg, void *buffer, size_t len)
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

