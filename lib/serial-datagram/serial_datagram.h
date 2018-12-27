#ifndef SERIAL_DATAGRAM_H
#define SERIAL_DATAGRAM_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_DATAGRAM_CRC_START 0x00000000UL

/* datagram receive callback function type */
typedef void (*serial_datagram_cb_t)(const void *dtgrm, size_t len, void *arg);

/**
 * Datagram reception handler
 * Stores datagram assembly buffer, handling callback and protocol state.
 */
typedef struct {
    uint8_t *buffer;
    size_t size;
    uint32_t write_index;
    serial_datagram_cb_t callback_fn;
    void *callback_arg;
    bool error_flag;
    bool esc_flag;
} serial_datagram_rcv_handler_t;


/**
 * Send a datagram
 */
void serial_datagram_send(const void *dtgrm, size_t len,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg);

/**
 * Send datagram chunk
 *
 * This funciton should be used when the datagram needs to be assembled from
 * multiple buffers.
 */
void serial_datagram_send_chunk(const void *dtgrm, size_t len, uint32_t *crc,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg);
/**
 * Send datagram end sequence: [CRC32][END]
 */
void serial_datagram_send_end(uint32_t crc,
        void (*send_fn)(void *arg, const void *p, size_t len), void *sendarg);

/**
 * Setup datagram receive handler
 *
 * The size of the buffer passed as argument minus 4bytes for the CRC determines
 *  the maximum datagram size that can be received.
 * The callback function is called once for each correctly received datagram
 *  with a pointer to it.
 */
void serial_datagram_rcv_handler_init(serial_datagram_rcv_handler_t *h,
        void *buffer, size_t size, serial_datagram_cb_t cb_fn, void *cb_arg);

/**
 * Receive bytes of a datagram
 *
 * This function calls the callback defined in the handler_init function
 * as soon as a complete, correct datagram is received.
 * If there is a reception error, an error code is returned once and all bytes
 * until the start of the next datagram are ignored without further error codes.
 */
int serial_datagram_receive(serial_datagram_rcv_handler_t *h, const void *in,
        size_t len);

/** Error codes for serial_datagram_receive */
#define SERIAL_DATAGRAM_RCV_NO_ERROR            0
#define SERIAL_DATAGRAM_RCV_DATAGRAM_TOO_LONG   1
#define SERIAL_DATAGRAM_RCV_CRC_MISMATCH        2
#define SERIAL_DATAGRAM_RCV_PROTOCOL_ERROR      3


#ifdef __cplusplus
}
#endif

#endif /* SERIAL_DATAGRAM_H */
