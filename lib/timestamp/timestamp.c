#include "timestamp.h"


int32_t timestamp_duration_us(timestamp_t t1, timestamp_t t2)
{
    return (t2 - t1);
}

float timestamp_duration_s(timestamp_t t1, timestamp_t t2)
{
    return (float)timestamp_duration_us(t1, t2)/1000000.f;
}


int64_t ltimestamp_duration_us(ltimestamp_t t1, ltimestamp_t t2)
{
    return (t2 - t1);
}

float ltimestamp_duration_s(ltimestamp_t t1, ltimestamp_t t2)
{
    return (float)ltimestamp_duration_us(t1, t2)/1000000.f;
}
