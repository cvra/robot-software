#include "timestamp.h"

static unix_timestamp_t unix_reference = {.s=0, .us=0};
static int32_t local_reference = 0;

int32_t timestamp_unix_to_local_us(unix_timestamp_t ts)
{
    ts.s -= unix_reference.s;
    ts.us -= unix_reference.us;
    return ts.s * 1000 + ts.us + local_reference;
}

unix_timestamp_t timestamp_local_us_to_unix(int32_t ts)
{
    unix_timestamp_t result;

    ts -= local_reference;
    result.s = ts / 1000 + unix_reference.s;
    result.us = ts % 1000 + unix_reference.us;

    if (result.us >= 1000) {
        result.s += 1;
        result.us -= 1000;
    }

    return result;
}

void timestamp_set_reference(unix_timestamp_t unix_ts, int32_t local_ts)
{
    unix_reference = unix_ts;
    local_reference = local_ts;
}
