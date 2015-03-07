#include <string.h>
#include "service_call.h"

void service_call_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem, uint8_t *buffer, size_t buffer_size, const char *method_name, int param_count)
{
    cmp_mem_access_init(cmp, mem, buffer, buffer_size);
    cmp_write_str(cmp, method_name, strlen(method_name));
    cmp_write_map(cmp, param_count);
}

size_t service_call_process(const uint8_t *buffer, size_t buffer_size,
                            uint8_t *output_buffer, size_t output_buffer_size,
                            service_call_method *callbacks, int callbacks_len)
{
    cmp_ctx_t cmp, out_cmp;
    cmp_mem_access_t mem, out_mem;
    char *method_name;
    uint32_t method_name_len;
    uint32_t argc;
    int i;

    cmp_mem_access_ro_init(&cmp, &mem, buffer, buffer_size);

    cmp_mem_access_init(&out_cmp, &out_mem, output_buffer, output_buffer_size);

    cmp_read_str_size(&cmp, &method_name_len);
    size_t name_pos = cmp_mem_access_get_pos(&mem);
    cmp_mem_access_set_pos(&mem, name_pos + method_name_len); // jump name string
    method_name = cmp_mem_access_get_ptr_at_pos(&mem, name_pos);
    cmp_read_map(&cmp, &argc);

    for (i = 0; i < callbacks_len; ++i) {
        if (!strncmp(method_name, callbacks[i].name, method_name_len)) {
            callbacks[i].cb(argc, &cmp, &out_cmp);
            break;
        }
    }

    return out_mem.index;
}
