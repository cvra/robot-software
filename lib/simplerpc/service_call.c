#include <string.h>
#include "service_call.h"

void service_call_write_header(cmp_ctx_t *cmp,
                              cmp_mem_access_t *mem,
                              uint8_t *buffer,
                              size_t buffer_size,
                              const char *method_name)
{
    cmp_mem_access_init(cmp, mem, buffer, buffer_size);
    cmp_write_array(cmp, 2);
    cmp_write_str(cmp, method_name, strlen(method_name));
}

size_t service_call_process(const uint8_t *buffer,
                            size_t buffer_size,
                            uint8_t *output_buffer,
                            size_t output_buffer_size,
                            struct service_call_method_s *callbacks,
                            int callbacks_len)
{
    cmp_ctx_t cmp, out_cmp;
    cmp_mem_access_t mem, out_mem;
    char *method_name;
    uint32_t method_name_len;
    uint32_t argc = 0;
    int i;

    cmp_mem_access_ro_init(&cmp, &mem, buffer, buffer_size);

    cmp_mem_access_init(&out_cmp, &out_mem, output_buffer, output_buffer_size);

    cmp_read_array(&cmp, &argc);

    if (argc != 2) {
        return 0;
    }

    if (cmp_read_str_size(&cmp, &method_name_len) == false) {
        return 0;
    }

    size_t name_pos = cmp_mem_access_get_pos(&mem);
    cmp_mem_access_set_pos(&mem, name_pos + method_name_len); // jump name string
    method_name = cmp_mem_access_get_ptr_at_pos(&mem, name_pos);

    for (i = 0; i < callbacks_len; ++i) {
        if (!strncmp(method_name, callbacks[i].name, method_name_len)) {
            if (callbacks[i].cb(callbacks[i].arg, &cmp, &out_cmp)) {
                break;
            } else {
                return 0;
            }
        }
    }

    if (cmp_mem_access_get_pos(&out_mem) == 0) {
        cmp_write_nil(&out_cmp);
    }
    return cmp_mem_access_get_pos(&out_mem);
}
