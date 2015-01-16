#include "panic_log.h"
#include <string.h>

extern char __panic_log;
extern char __panic_log_len;

void panic_log_write(const char *msg)
{
    char *buffer = &__panic_log;
    size_t len = (size_t)&__panic_log_len;

    strncpy(buffer, msg, len);

    // Terminates the buffer
    buffer[len - 1] = '\0';
}

const char *panic_log_read(void)
{
    return &__panic_log;
}
