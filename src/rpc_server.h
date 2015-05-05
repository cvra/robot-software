#ifndef RPC_SERVER_H
#define RPC_SERVER_H


#define RPC_SERVER_PORT 20001
#define MSG_SERVER_PORT 20000

#ifdef __cplusplus
extern "C" {
#endif

/** Starts the Remote Procedure Call server. */
void rpc_server_init(void);

/** Packs a buffer in a serial datagram, sends it to the given address and gets it back.
 *
 * @param [in] input_buffer Buffer to transmit
 * @param [in] input_buffer_size Length of input_buffer, in bytes.
 * @param [out] output_buffer Buffer in which received data will be put.
 * @param [out] output_buffer_size Size of output_buffer in bytes.
 * @param [in] addr Pointer to a struct filled with the IP address of the
 * server, for example via DNS or IP4_ADDR.
 * @param [in] port TCP port to connect to.
 *
 * @return Number of bytes written to output_buffer or -1 in case of error.
 */
size_t rpc_transmit(uint8_t *input_buffer, size_t input_buffer_size,
                    uint8_t *output_buffer, size_t output_buffer_size,
                    ip_addr_t *addr, uint16_t port);

void message_transmit(uint8_t *input_buffer, size_t input_buffer_size, ip_addr_t *addr, uint16_t port);

void message_server_init(void);


#ifdef __cplusplus
}
#endif


#endif
