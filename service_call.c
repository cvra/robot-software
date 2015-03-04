#include <string.h>
#include "service_call.h"

void service_call_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem, uint8_t *buffer, size_t buffer_size, const char *method_name, int param_count)
{
    cmp_mem_access_init(cmp, mem, buffer, buffer_size);
    cmp_write_str(cmp, method_name, strlen(method_name));
    cmp_write_map(cmp, param_count);
}
