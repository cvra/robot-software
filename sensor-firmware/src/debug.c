#include <hal.h>
#include <chprintf.h>
#include <stdarg.h>
#include <string.h>
#include <error/error.h>

#define OUTPUT_STREAM ((BaseSequentialStream*)&SD2)

MUTEX_DECL(log_lock);

static const char* get_thread_name(void)
{
    const char* thread_name;

    thread_name = chRegGetThreadNameX(chThdGetSelfX());
    if (thread_name == NULL) {
        thread_name = "unknown";
    }

    return thread_name;
}

static void log_message(struct error* e, ...)
{
    va_list va;
    chMtxLock(&log_lock);

    va_start(va, e);

    /* Print time */
    uint32_t ts = TIME_I2MS(chVTGetSystemTime());
    uint32_t s = ts / 1000;
    uint32_t ms = ts % 1000;
    chprintf(OUTPUT_STREAM, "[%4d.%03d]\t", s, ms);

    /* Print location. */
    chprintf(OUTPUT_STREAM, "%s:%d\t", strrchr(e->file, '/') + 1, e->line);

    /* Print current thread */
    chprintf(OUTPUT_STREAM, "%s\t", get_thread_name());

    /* Print severity message */
    chprintf(OUTPUT_STREAM, "%s\t", error_severity_get_name(e->severity));

    /* Print message */
    chvprintf(OUTPUT_STREAM, e->text, va);

    chprintf(OUTPUT_STREAM, "\n");

    va_end(va);

    chMtxUnlock(&log_lock);
}

static const SerialConfig debug_serial_config = {
    921600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0};

void debug_init(void)
{
    sdStart(&SD2, &debug_serial_config);

    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    /* Disabled by default to avoid verbose messages. */
    // error_register_debug(log_message);
}

void panic_handler(void)
{
    palSetPad(GPIOA, GPIOA_PIN0);
    palSetPad(GPIOA, GPIOA_PIN6);
    while (1) {
    }
}
