#include "panic_log.h"
#include <string.h>
#include <crc/crc32.h>

extern char __panic_log;
extern char __panic_log_len;

void panic_log_write(const char *msg)
{
    uint32_t *crc = (uint32_t *)&__panic_log;
    char *buffer = &__panic_log + sizeof(uint32_t);
    size_t len = (size_t)&__panic_log_len - sizeof(uint32_t);

    strncpy(buffer, msg, len);
    *crc = crc32(0, buffer, len);

    // Terminates the buffer
    buffer[len - 1] = '\0';
}

const char *panic_log_read(void)
{
    uint32_t *crc = (uint32_t *)&__panic_log;
    char *buffer = &__panic_log + sizeof(uint32_t);
    size_t len = (size_t)&__panic_log_len - sizeof(uint32_t);

    if (*crc == crc32(0, buffer, len)) {
        return buffer;
    }
    return NULL;
}
