#include "panic_log.h"
#include <string.h>
#include <crc/crc32.h>

__attribute__ ((section(".noinit")))
__attribute__ ((aligned(4)))
char panic_log[1024];
const size_t panic_log_len = sizeof(panic_log);

void panic_log_write(const char *msg)
{
    uint32_t *crc = (uint32_t *)&panic_log[0];
    char *buffer = &panic_log[0] + sizeof(uint32_t);
    size_t len = sizeof(panic_log) - sizeof(uint32_t);

    strncpy(buffer, msg, len);
    // Terminates the buffer
    size_t msg_len = strnlen(msg, len);
    if (msg_len >= len) {
        buffer[len - 1] = '\0';
    } else {
        buffer[msg_len] = '\0';
    }

    *crc = crc32(0, buffer, len);
}

const char *panic_log_read(void)
{
    uint32_t *crc = (uint32_t *)&panic_log[0];
    char *buffer = &panic_log[0] + sizeof(uint32_t);
    size_t len = sizeof(panic_log) - sizeof(uint32_t);

    if (*crc == crc32(0, buffer, len)) {
        return buffer;
    }
    return NULL;
}
