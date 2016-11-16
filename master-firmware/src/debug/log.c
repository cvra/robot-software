#include <hal.h>
#include <chprintf.h>
#include <ff.h>
#include <stdio.h>
#include <string.h>
#include "timestamp/timestamp.h"
#include <error/error.h>

#include "log.h"
#include "usbconf.h"

#define OUTPUT_STREAM ((BaseSequentialStream*)&SD3)


MUTEX_DECL(log_lock);

static FIL logfile_fp;
static bool log_file_enabled = false;

static void vuart_log_message(struct error *e, va_list args);
static void vlogfile_log_message(struct error *e, va_list args);

static void log_message(struct error *e, ...)
{
    va_list va;

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

static void vuart_log_message(struct error *e, va_list args)
{
    const char *thread_name = NULL;
    if (ch.rlist.r_current) {
        thread_name = ch.rlist.r_current->p_name;
    }

    /* Print time */
    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;
    chprintf(OUTPUT_STREAM, "[%4d.%06d]\t", s, us);

    /* Print location. */
    chprintf(OUTPUT_STREAM, "%s:%d\t", strrchr(e->file, '/') + 1, e->line);

    /* Print current thread */
    if (thread_name != NULL) {
        chprintf(OUTPUT_STREAM, "%s\t", thread_name);
    }

    /* Print severity message */
    chprintf(OUTPUT_STREAM, "%s\t", error_severity_get_name(e->severity));

    /* Print message */
    chvprintf(OUTPUT_STREAM, e->text, args);

    chprintf(OUTPUT_STREAM, "\n");
}

static void vlogfile_log_message(struct error *e, va_list args)
{
    static char buffer[256];
    UINT dummy;
    const char *thread_name = NULL;

    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;

    /* Write severity */
    f_write(&logfile_fp, error_severity_get_name(e->severity),
            strlen(error_severity_get_name(e->severity)), &dummy);

    /* Write time stamp */
    snprintf(buffer, sizeof(buffer), "[%4ld.%06ld]\t", s, us);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);
    f_write(&logfile_fp, "\t", 1, &dummy);

    /* Write filename line. */
    snprintf(buffer, sizeof(buffer), "%s:%d\t", strrchr(e->file, '/') + 1, e->line);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);

    /* Write current thread. */
    if (ch.rlist.r_current) {
        thread_name = ch.rlist.r_current->p_name;
    } else {
        thread_name = "unknown";
    }
    f_write(&logfile_fp, thread_name, strlen(thread_name), &dummy);
    f_write(&logfile_fp, "\t", 1, &dummy);

    /* Write error message */
    vsnprintf(buffer, sizeof(buffer), e->text, args);
    f_write(&logfile_fp, buffer, strlen(buffer), &dummy);

    /* Forces the data to be written. */
    f_write(&logfile_fp, "\n", 1, &dummy);
    f_sync(&logfile_fp);
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
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    /* Disabled by default to avoid verbose messages. */
    // error_register_debug(log_message);

    if (try_sd_card_mount()) {
        log_file_enabled = true;
        NOTICE("Logging on SD card enabled.");
    } else {
        WARNING("Logging on SD card disabled, logs will be lost...");
    }
}
