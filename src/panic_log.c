#include <ch.h>
#include <chprintf.h>
#include <memstreams.h>
#include <crc/crc32.h>
#include <string.h>
#include "panic_log.h"

__attribute__ ((section(".noinit")))
char panic_log[500];
__attribute__ ((section(".noinit")))
uint32_t panic_log_crc;

bool panic_log_stream_is_initialized = false;
MemoryStream panic_log_stream;

void panic_log_printf(const char *fmt, ...)
{
    if (!panic_log_stream_is_initialized) {
        // clear & renitialize panic log
        panic_log_clear();
    }

    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&panic_log_stream, fmt, ap);
    va_end(ap);

    panic_log_crc = crc32(0, &panic_log[0], sizeof(panic_log));
}

void panic_log_write(const char *msg)
{
    panic_log_printf("%s", msg);
}

const char *panic_log_read(void)
{
    if (panic_log_crc == crc32(0, &panic_log[0], sizeof(panic_log))) {
        return &panic_log[0];
    }
    return NULL;
}

void panic_log_clear(void)
{
    memset(&panic_log[0], 0, sizeof(panic_log));
    panic_log_crc = 0;
    // initialize stream object with buffer
    msObjectInit(&panic_log_stream, (uint8_t *)&panic_log[0],
        sizeof(panic_log) - 1, 0); // size - 1 for null terminated message
    panic_log_stream_is_initialized = true;
}

