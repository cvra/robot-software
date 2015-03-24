#ifndef MESSAGE_H
#define MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <cmp_mem_access/cmp_mem_access.h>

typedef struct {
    const char *name;
    void (*cb)(void *, int, cmp_ctx_t *);
    void *arg;
} message_method_t;

/** @brief Prepares a message
 *
 * This function creates a cmp context for a message with the given number of
 * argument and name.
 * This context can then be used for writing the arguments.
 *
 * @param [in] buffer The output buffer to use.
 * @param [in] buffer_size Size of buffer in bytes.
 * @param [in] method_name Name of the RPC call.
 * @param [in] param_count Number of arguments to the service call.
 * @param [out] cmp Pointer to the cmp instance to use.
 * @param [out] mem Pointer to the cmp_mem_access_t instance to use.
 */
void message_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem,
                    uint8_t *buffer, size_t buffer_size,
                    const char *method_name, int param_count);


/** @brief Processes a buffer and calls the correct callback.
 *
 * @param [in] buffer The buffer containing the encoded message.
 * @param [in] buffer_size Length of the buffer in bytes.
 * @param [in] callbacks Array of all possible callback function and their associated types.
 * @param [in] callbacks_len Number of items in callbacks.
 */
void message_process(uint8_t *buffer, size_t buffer_size,
                     message_method_t *callbacks, unsigned int callbacks_len);


#ifdef __cplusplus
}
#endif

#endif
