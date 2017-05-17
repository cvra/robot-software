#ifndef UART_STREAM_H
#define UART_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch.h>

void uart_stream_start(BaseSequentialStream *dev);

#ifdef __cplusplus
}
#endif

#endif /* UART_STREAM_H */
