#include "message.h"
#include <string.h>

void message_write_header(cmp_ctx_t *cmp,
                          cmp_mem_access_t *mem,
                          uint8_t *buffer,
                          size_t buffer_size,
                          const char *method_name)
{
    cmp_mem_access_init(cmp, mem, buffer, buffer_size);
    cmp_write_array(cmp, 2);
    cmp_write_str(cmp, method_name, strlen(method_name));
}

void message_process(uint8_t *buffer,
                     size_t buffer_size,
                     struct message_method_s *callbacks,
                     unsigned int callbacks_len)
{
    unsigned int i;
    uint32_t argc = 0;

    cmp_ctx_t cmp;
    cmp_mem_access_t mem;

    char *method_name;
    uint32_t method_name_len;

    cmp_mem_access_ro_init(&cmp, &mem, buffer, buffer_size);
    cmp_read_array(&cmp, &argc);

    if (argc != 2) {
        return;
    }

    if (cmp_read_str_size(&cmp, &method_name_len) == false) {
        return;
    }

    /* Read string in place */
    size_t name_pos = cmp_mem_access_get_pos(&mem);
    cmp_mem_access_set_pos(&mem, name_pos + method_name_len);
    method_name = cmp_mem_access_get_ptr_at_pos(&mem, name_pos);

    for (i = 0; i < callbacks_len; ++i) {
        if (!strncmp(method_name, callbacks[i].name, method_name_len)) {
            callbacks[i].cb(callbacks[i].arg, &cmp);
            break;
        }
    }
}

