#include <hal.h>
#include <chprintf.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>
#include "timestamp/timestamp.h"
#include <error/error.h>
#include <parameter/parameter.h>
#include "main.h"
#include "chibios-syscalls/stdio_lock.h"

#include "log.h"
#include "usbconf.h"
#include "gui.h"

#define OUTPUT_STREAM ((BaseSequentialStream*)&SD7)

MUTEX_DECL(log_lock);

static FIL logfile_fp;
static bool log_file_enabled = false;

static void vuart_log_message(struct error* e, va_list args);
static void vlogfile_log_message(struct error* e, va_list args);
static void vpanic_message(struct error* e, va_list args);
static unsigned int get_level_parameter(parameter_t* p);

static struct {
    parameter_namespace_t ns;
    struct {
        parameter_namespace_t ns;
        parameter_t level;
        char level_buf[10];
    } uart, sdcard;
} params;

static void log_message(struct error* e, ...)
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

    if (log_file_enabled) {
        va_start(va, e);
        vlogfile_log_message(e, va);
        va_end(va);
    }

    chMtxUnlock(&log_lock);
}

static const char* get_thread_name(void)
{
    const char* thread_name;

    thread_name = chRegGetThreadNameX(chThdGetSelfX());
    if (thread_name == NULL) {
        thread_name = "unknown";
    }

    return thread_name;
}

static unsigned int get_level_parameter(parameter_t* p)
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

static void vuart_log_message(struct error* e, va_list args)
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

static void vlogfile_log_message(struct error* e, va_list args)
{
    if (get_level_parameter(&params.sdcard.level) > e->severity) {
        return;
    }

    if (!palReadPad(GPIOA, GPIOA_SD_DETECT)) {
        /* Indicate SD activity */
        palTogglePad(GPIOB, GPIOB_LED_SD);
    } else {
        /* Card was removed */
        return;
    }

    static char buffer[256];
    UINT dummy;
    const char* thread_name = NULL;

    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;

    /* Write severity */
    f_write(&logfile_fp, error_severity_get_name(e->severity),
            strlen(error_severity_get_name(e->severity)), &dummy);

    /* Write time stamp */
    stdio_lock();
    snprintf(buffer, sizeof(buffer), "[%4ld.%06ld]\t", s, us);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);
    f_write(&logfile_fp, "\t", 1, &dummy);

    /* Write filename line. */
    snprintf(buffer, sizeof(buffer), "%s:%d\t", strrchr(e->file, '/') + 1, e->line);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);

    /* Write thread name */
    thread_name = get_thread_name();
    f_write(&logfile_fp, thread_name, strlen(thread_name), &dummy);
    f_write(&logfile_fp, "\t", 1, &dummy);

    /* Write error message */
    vsnprintf(buffer, sizeof(buffer), e->text, args);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);
    stdio_unlock();

    /* Forces the data to be written. */
    f_write(&logfile_fp, "\n", 1, &dummy);
    f_sync(&logfile_fp);
}

static void vpanic_message(struct error* e, va_list args)
{
    static char buffer[256];
    vsnprintf(buffer, sizeof(buffer), e->text, args);
    chSysHalt(buffer);
}

static bool try_sd_card_mount(void)
{
    FRESULT err;
    err = f_open(&logfile_fp, "/log.txt", FA_OPEN_ALWAYS | FA_WRITE);

    if (err != FR_OK) {
        WARNING("Cannot create log file.");
        return false;
    }

    /* Seek to end of file */
    err = f_lseek(&logfile_fp, f_size(&logfile_fp));

    if (err != FR_OK) {
        WARNING("Cannot seek to end of log file.");
        return false;
    }

    return true;
}

void log_init(void)
{
    parameter_namespace_declare(&params.ns, &global_config, "logging");

    parameter_namespace_declare(&params.sdcard.ns, &params.ns, "sdcard");
    parameter_string_declare_with_default(&params.sdcard.level,
                                          &params.sdcard.ns,
                                          "level",
                                          params.sdcard.level_buf,
                                          sizeof(params.sdcard.level_buf),
                                          "notice");
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

    if (try_sd_card_mount()) {
        log_file_enabled = true;
        NOTICE("Logging on SD card enabled.");
    } else {
        WARNING("Logging on SD card disabled, logs will be lost...");
    }
}
