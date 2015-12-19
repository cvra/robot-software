#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <cmp/cmp.h>
#include "cmp_mem_access.h"

int main(void)
{
    cmp_mem_access_t mem;
    cmp_ctx_t cmp;
    char buffer[128];

    // initialize memory access
    cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));

    // write to the buffer
    cmp_write_map(&cmp, 1);
    cmp_write_str(&cmp, "test", strlen("test"));
    cmp_write_int(&cmp, 42);

    // print the buffer contents in hex
    printf("buffer contents:\n");
    int serialized_len = cmp_mem_access_get_pos(&mem);
    int i;
    for (i = 0; i < serialized_len; i++) {
        printf("%02x ", (uint8_t)buffer[i]);
    }

    // reinitialize memory access for read operation
    cmp_mem_access_ro_init(&cmp, &mem, buffer, sizeof(buffer));

    // read back from buffer
    uint32_t map_size;
    cmp_read_map(&cmp, &map_size);
    printf("\n\nread map of length %d\n", map_size);
    char str_buf[10];
    uint32_t str_buf_sz = sizeof(str_buf);
    cmp_read_str(&cmp, str_buf, &str_buf_sz);
    int32_t val;
    cmp_read_int(&cmp, &val);
    printf("read:  {%s: %d}\n", str_buf, val);

    return 0;
}
