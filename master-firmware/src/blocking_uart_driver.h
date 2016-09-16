#ifndef BLOCKING_UART_DRIVER_H
#define BLOCKING_UART_DRIVER_H

#include <ch.h>
#include <hal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const struct BaseSequentialStreamVMT blocking_uart_put_vmt;

typedef struct {
    const struct BaseSequentialStreamVMT *vmt;
    USART_TypeDef *dev;
} BlockingUARTDriver;

msg_t blocking_uart_put(void *instance, uint8_t b);
msg_t blocking_uart_get(void *instance);
size_t blocking_uart_write(void *instance, const uint8_t *bp, size_t n);
size_t blocking_uart_read(void *instance, uint8_t *bp, size_t n);

void blocking_uart_init(BlockingUARTDriver *driver, USART_TypeDef *uart, uint32_t baud);

#ifdef __cplusplus
}
#endif

#endif /* BLOCKING_UART_DRIVER_H */