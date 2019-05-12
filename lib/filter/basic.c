#include "filter/basic.h"

float filter_limit(float value, float min, float max)
{
    if (value > max)
        return max;
    else if (value < min)
        return min;
    else
        return value;
}

float filter_limit_sym(float value, float limit)
{
    return filter_limit(value, -limit, limit);
}
