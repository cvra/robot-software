#include "trace_points.h"

#undef C
#define C(x) #x,

const char *trace_point_names[] = {
    TRACE_POINTS
};
