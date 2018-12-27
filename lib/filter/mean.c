#include <stdlib.h>
#include "mean.h"


float mean(float* set, size_t n)
{
    float ret = 0.0f;
    int i;

    for (i = 0; i < n; i++) {
        ret += set[i];
    }

    return ret / n;
}


int mean_int16(int16_t* set, size_t n)
{
    int ret = 0;
    int i;

    for (i = 0; i < n; i++) {
        ret += set[i];
    }

    return ret / n;
}
