#include <error/error.h>
#include "unix_timestamp.h"

static unix_timestamp_t unix_reference = {.s = 0, .us = 0};
static int32_t local_reference = 0;

int32_t timestamp_unix_to_local_us(unix_timestamp_t ts)
{
    ts.s -= unix_reference.s;
    ts.us -= unix_reference.us;
    return ts.s * 1000000 + ts.us + local_reference;
}

unix_timestamp_t timestamp_local_us_to_unix(int32_t ts)
{
    unix_timestamp_t result;

    ts -= local_reference;
    result.s = ts / 1000000 + unix_reference.s;
    result.us = ts % 1000000 + unix_reference.us;

    if (result.us >= 1000000) {
        result.s += 1;
        result.us -= 1000000;
    }

    return result;
}

void timestamp_set_reference(unix_timestamp_t unix_ts, int32_t local_ts)
{
    DEBUG("NTP time update: %d is: %d.%06d", local_ts, unix_ts.s, unix_ts.us);
    unix_reference = unix_ts;
    local_reference = local_ts;
}

int timestamp_unix_compare(unix_timestamp_t a, unix_timestamp_t b)
{
    if (a.s < b.s) {
        return -1;
    } else if (a.s == b.s) {
        if (a.us < b.us) {
            return -1;
        } else if (a.us == b.us) {
            return 0;
        } else {
            return 1;
        }
    } else {
        return 1;
    }
}
