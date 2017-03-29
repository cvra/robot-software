#include <string.h>
#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include "chstream_error_logger.h"

static const char *get_thread_name(void)
{
    const char *thread_name;

    thread_name = chRegGetThreadNameX(chThdGetSelfX());
    if (thread_name == NULL) {
        thread_name = "unknown";
    }

    return thread_name;
}

void error_chstream_logger_write(BaseSequentialStream *stream, struct error *e, va_list args)
{
    /* Print time */
    uint32_t ts = ST2MS(chVTGetSystemTime());
    uint32_t s = ts / 1000;
    uint32_t ms = ts % 1000;

    chprintf(stream, "[%4d.%03d]\t", s, ms);

    /* Print location. */
    chprintf(stream, "%s:%d\t", strrchr(e->file, '/') + 1, e->line);

    /* Print current thread */
    chprintf(stream, "%s\t", get_thread_name());

    /* Print severity message */
    chprintf(stream, "%s\t", error_severity_get_name(e->severity));

    /* Print message */
    chvprintf(stream, e->text, args);

    chprintf(stream, "\n");
}
