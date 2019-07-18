#ifndef TRACE_POINTS_H
#define TRACE_POINTS_H

#include <trace/trace.h>

#define TRACE_POINTS               \
    C(TRACE_POINT_PANIC)           \
    C(TRACE_POINT_CONTEXT_SWITCH)  \
    C(TRACE_POINT_RPC_MESSAGE_RCV) \
    C(TRACE_POINT_RPC_MESSAGE_SEND)

/* List of all trace points in numerical format. */
#undef C
#define C(x) x,
enum {
    TRACE_POINTS
};

#endif /* TRACE_POINTS_H */
