#ifndef TRACE_POINTS_H
#define TRACE_POINTS_H

#include "trace.h"

#define TRACE_POINTS \
    C(TRACE_POINT_BOOT) \

/* List of all trace points in numerical format. */
#undef C
#define C(x) x,
enum {
    TRACE_POINTS
};

#endif /* TRACE_POINTS_H */
