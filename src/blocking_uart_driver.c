#include <stdint.h>
#include <ch.h>
#include <hal.h>
#include "blocking_uart_driver.h"

// Virtual Methods Table
const struct BaseSequentialStreamVMT blocking_uart_put_vmt = {
    blocking_uart_write,
    blocking_uart_read,
    blocking_uart_put,
    blocking_uart_get,
};

msg_t blocking_uart_put(void *instance, uint8_t b)
{
    USART_TypeDef *uart = ((BlockingUARTDriver *)instance)->dev;
    // wait until TX register empty
    while ((uart->ISR & USART_ISR_TXE) == 0);
    // write data
    uart->TDR = b;
    return 0;
}

msg_t blocking_uart_get(void *instance)
{
    USART_TypeDef *uart = ((BlockingUARTDriver *)instance)->dev;
    // wait until RX register not empty
    while ((uart->ISR & USART_ISR_RXNE) == 0);
    // return byte
    return uart->RDR & USART_RDR_RDR;
}

size_t blocking_uart_write(void *instance, const uint8_t *bp, size_t n)
{
    USART_TypeDef *uart = ((BlockingUARTDriver *)instance)->dev;
    size_t i;
    for (i = 0; i < n; i++) {
        // wait until TX register empty
        while ((uart->ISR & USART_ISR_TXE) == 0);
        // write data
        uart->TDR = bp[i];
    }
    return n;
}

size_t blocking_uart_read(void *instance, uint8_t *bp, size_t n)
{
    USART_TypeDef *uart = ((BlockingUARTDriver *)instance)->dev;
    size_t i;
    for (i = 0; i < n; i++) {
        // wait until RX register not empty
        while ((uart->ISR & USART_ISR_RXNE) == 0);
        // read byte
        bp[i] = (uint8_t) uart->RDR;
    }
    return n;
}

void blocking_uart_init(BlockingUARTDriver *driver, USART_TypeDef *uart, uint32_t baud)
{
    driver->dev = uart;
    driver->vmt = &blocking_uart_put_vmt;

    uint32_t clock = STM32_PCLK1;

    chSysLock();

#if defined(USART1)
    if (uart == USART1) {
        rccEnableUSART1(FALSE); // FALSE = disable low power flag
        clock = STM32_PCLK2;
    }
#endif
#if defined(USART2)
    if (uart == USART2) {
        rccEnableUSART2(FALSE);
    }
#endif
#if defined(USART3)
    if (uart == USART3) {
        rccEnableUSART3(FALSE);
    }
#endif
#if defined(USART4)
    if (uart == USART4) {
        rccEnableUSART4(FALSE);
    }
#endif
#if defined(USART5)
    if (uart == USART5) {
        rccEnableUSART5(FALSE);
    }
#endif
#if defined(USART6)
    if (uart == USART6) {
        clock = STM32_PCLK2;
        rccEnableUSART6(FALSE);
    }
#endif

    // baud rate, (with rounding)
    uart->BRR = ((2 * clock) + baud) / (2 * baud);

    // 1 stop bit
    uart->CR2 = 0;

    // no flow control
    uart->CR3 = 0;

    // tx/rx, 8bits, no parity & enable uart
    uart->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;

    // clear all flags
    uart->ICR = 0xFFFFFFFF;

    chSysUnlock();
}
