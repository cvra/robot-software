#ifndef TRACE_POINTS_H
#define TRACE_POINTS_H

#define TRACE_POINTS \
    C(TRACE_POINT_0) \
    C(TRACE_POINT_1) \
    C(TRACE_POINT_2) \

/* List of all trace points in numerical format. */
#undef C
#define C(x) x,
enum {
    TRACE_POINTS
};

#endif /* TRACE_POINTS_H */
