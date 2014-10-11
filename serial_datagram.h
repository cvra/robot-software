#ifndef SERIAL_DATAGRAM_H
#define SERIAL_DATAGRAM_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Datagram reception handler
 * Stores datagram assembly buffer, handling callback and protocol state.
 */
typedef struct {
    char *buffer;
    size_t size;
    int write_index;
    void (*callback_fn)(const char *dtgrm, size_t len);
    bool error_flag;
    bool esc_flag;
} serial_datagram_rcv_handler_t;


/**
 * Send a datagram
 */
void serial_datagram_send(const char *dtgrm, size_t len,
        void (*send_fn)(const char *p, size_t len));

/**
 * Setup datagram receive handler
 *
 * The size of the buffer passed as argument minus 4bytes for the CRC determines
 *  the maximum datagram size that can be received.
 * The callback function is called once for each correctly received datagram
 *  with a pointer to it.
 */
void serial_datagram_rcv_handler_init(serial_datagram_rcv_handler_t *h,
        char *buffer, size_t size, void (*cb)(const char *dtgrm, size_t len));

/**
 * Receive bytes of a datagram
 *
 * This function calls the callback defined in the handler_init function
 * as soon as a complete, correct datagram is received.
 * If there is a reception error, an error code is returned once and all bytes
 * until the start of the next datagram are ignored without further error codes.
 */
int serial_datagram_receive(serial_datagram_rcv_handler_t *h, const char *in,
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
