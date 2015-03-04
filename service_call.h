#ifndef SERVICE_CALL_H
#define SERVICE_CALL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <cmp_mem_access/cmp_mem_access.h>

#ifdef __cplusplus
extern "C" {
#endif

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
void service_call_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem, uint8_t *buffer, size_t buffer_size, const char *method_name, int param_count);

#ifdef __cplusplus
}
#endif
#endif
