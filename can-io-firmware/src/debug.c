#include <hal.h>
#include <chprintf.h>
#include <stdarg.h>

void debug_msg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
    va_end(ap);
}

static const SerialConfig debug_serial_config = {
    921600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};

void debug_init(void)
{
    palSetPadMode(GPIOA, GPIOA_PIN9, PAL_MODE_ALTERNATE(7));
    palSetPadMode(GPIOA, GPIOA_PIN10, PAL_MODE_ALTERNATE(7));
    sdStart(&SD1, &debug_serial_config);
}

void panic_handler(void)
{
    palSetPad(GPIOA, GPIOA_LED);
    while(1);
}
