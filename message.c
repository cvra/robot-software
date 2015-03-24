#include "message.h"
#include <string.h>

void message_encode(cmp_ctx_t *cmp, cmp_mem_access_t *mem,
                    uint8_t *buffer, size_t buffer_size,
                    const char *method_name, int param_count)
{
    cmp_mem_access_init(cmp, mem, buffer, buffer_size);
    cmp_write_array(cmp, param_count + 1);
    cmp_write_str(cmp, method_name, strlen(method_name));
}

void message_process(uint8_t *buffer, size_t buffer_size,
                     message_method_t *callbacks, unsigned int callbacks_len)
{
    unsigned int i;
    uint32_t argc;

    cmp_ctx_t cmp;
    cmp_mem_access_t mem;

    char *method_name;
    uint32_t method_name_len;

    cmp_mem_access_ro_init(&cmp, &mem, buffer, buffer_size);
    cmp_read_array(&cmp, &argc);

    /* Read string in place */
    cmp_read_str_size(&cmp, &method_name_len);
    size_t name_pos = cmp_mem_access_get_pos(&mem);
    cmp_mem_access_set_pos(&mem, name_pos + method_name_len);
    method_name = cmp_mem_access_get_ptr_at_pos(&mem, name_pos);

    /* Do not count message name */
    argc --;

    for (i = 0; i < callbacks_len; ++i) {
        if (!strncmp(method_name, callbacks[i].name, method_name_len)) {
            callbacks[i].cb(callbacks[i].arg, argc, &cmp);
            break;
        }
    }
}

