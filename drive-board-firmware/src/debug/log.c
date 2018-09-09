#include <hal.h>
#include <chprintf.h>
#include <stdio.h>
#include <string.h>

#include <timestamp/timestamp.h>
#include <error/error.h>
#include <parameter/parameter.h>

#include "main.h"
#include "log.h"
#include "usbconf.h"

#define OUTPUT_STREAM ((BaseSequentialStream*)&SD7)


MUTEX_DECL(log_lock);

static void vuart_log_message(struct error *e, va_list args);
static void vpanic_message(struct error *e, va_list args);
static unsigned int get_level_parameter(parameter_t *p);

static struct {
    parameter_namespace_t ns;
    struct {
        parameter_namespace_t ns;
        parameter_t level;
        char level_buf[10];
    } uart, sdcard;
} params;

static void log_message(struct error *e, ...)
{
    va_list va;

    if (e->severity == ERROR_SEVERITY_ERROR) {
        va_start(va, e);
        vpanic_message(e, va);
        va_end(va);
    }

    chMtxLock(&log_lock);

    va_start(va, e);
    vuart_log_message(e, va);
    va_end(va);

    chMtxUnlock(&log_lock);
}

static const char *get_thread_name(void)
{
    const char *thread_name;

    thread_name = chRegGetThreadNameX(chThdGetSelfX());
    if (thread_name == NULL) {
        thread_name = "unknown";
    }

    return thread_name;
}

static unsigned int get_level_parameter(parameter_t *p)
{
    char buf[10];
    parameter_string_get(p, buf, sizeof(buf));
    unsigned result;

    if (!strcmp(buf, "debug")) {
        result = ERROR_SEVERITY_DEBUG;
    } else if (!strcmp(buf, "error")) {
        result = ERROR_SEVERITY_ERROR;
    } else if (!strcmp(buf, "notice")) {
        result = ERROR_SEVERITY_NOTICE;
    } else if (!strcmp(buf, "warning")) {
        result = ERROR_SEVERITY_WARNING;
    } else {
        /* If invalid, then display everything. */
        result = ERROR_SEVERITY_DEBUG;
    }

    return result;
}

static void vuart_log_message(struct error *e, va_list args)
{
    if (e->severity < get_level_parameter(&params.uart.level)) {
        return;
    }

    /* Print time */
    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;
    chprintf(OUTPUT_STREAM, "[%4d.%06d]\t", s, us);

    /* Print location. */
    chprintf(OUTPUT_STREAM, "%s:%d\t", strrchr(e->file, '/') + 1, e->line);

    /* Print current thread */
    chprintf(OUTPUT_STREAM, "%s\t", get_thread_name());

    /* Print severity message */
    chprintf(OUTPUT_STREAM, "%s\t", error_severity_get_name(e->severity));

    /* Print message */
    chvprintf(OUTPUT_STREAM, e->text, args);

    chprintf(OUTPUT_STREAM, "\n");
}

static void vpanic_message(struct error *e, va_list args)
{
    static char buffer[256];
    vsnprintf(buffer, sizeof(buffer), e->text, args);
    chSysHalt(buffer);
}

void log_init(void)
{
    parameter_namespace_declare(&params.ns, &global_config, "logging");

    parameter_namespace_declare(&params.uart.ns, &params.ns, "uart");
    parameter_string_declare_with_default(&params.uart.level,
                                          &params.uart.ns,
                                          "level",
                                          params.uart.level_buf,
                                          sizeof(params.uart.level_buf),
                                          "notice");

    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);
    error_register_debug(log_message);
}
