#include "trace_points.h"

#undef C
#define C(x) #x,

const char *event_names[] = {
    TRACE_POINTS
};
