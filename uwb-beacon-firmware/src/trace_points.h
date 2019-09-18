#ifndef TRACE_POINTS_H
#define TRACE_POINTS_H
#include <trace/trace.h>

#define TRACE_POINTS                      \
    C(TRACE_POINT_UWB_IRQ)                \
    C(TRACE_POINT_UWB_SEND_ADVERTISEMENT) \
    C(TRACE_POINT_UWB_TX_DONE)            \
    C(TRACE_POINT_UWB_RX)                 \
    C(TRACE_POINT_UWB_PROCESS_FRAME)

/* List of all trace points in numerical format. */
#undef C
#define C(x) x,
enum {
    TRACE_POINTS
};

#endif /* TRACE_POINTS_H */
