#ifndef SERVICE_CALL_H
#define SERVICE_CALL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cmp_mem_access/cmp_mem_access.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief A single RPC method. */
typedef struct {
    char *name; /**< Method name (eg: "goto_point") */
    /** Method handler.
     * First argument is number of args.
     * Second argument is a pointer to a MessagePack context containing the args.
     * Third argument is a pointer to a MessagePack context for output.
     */
    void (*cb)(int, cmp_ctx_t*, cmp_ctx_t *);
} service_call_method;

/** @brief Prepares a service call.
 *
 * This method creates a cmp context for a service call with the provided
 * method name and argument count.
 *
 * @param [in] buffer The output buffer to use.
 * @param [in] buffer_size Size of buffer in bytes.
 * @param [in] method_name Name of the RPC call.
 * @param [in] param_count Number of arguments to the service call.
 * @param [out] cmp Pointer to the cmp instance to use.
 * @param [out] mem Pointer to the cmp_mem_access_t instance to use.
 */
void service_call_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem,
                         uint8_t *buffer, size_t buffer_size,
                         const char *method_name, int param_count);

/** @brief Processes the a buffer and calls the appropriate callback.
 *
 * This function reads data from a buffer and calls the appropriate callback
 * from the given list based on its name.
 *
 * @param [in] buffer Input buffer containing the data to parse.
 * @param [in] buffer_size Size of buffer in bytes.
 * @param [in] output_buffer Buffer to write the result of the call to.
 * @param [in] output_buffer_size Size of output_buffer in bytes.
 * @param [in] callbacks Array of possible callbacks.
 * @param [in] callbacks
 */
void service_call_process(const uint8_t *buffer, size_t buffer_size,
                          uint8_t *output_buffer, size_t output_buffer_size,
                          service_call_method *callbacks, int callbacks_len);

#ifdef __cplusplus
}
#endif
#endif
