#include "timestamp.h"
#include <chrono>

int64_t timestamp_get_us()
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::time_point_cast<std::chrono::microseconds>(now)
        .time_since_epoch()
        .count();
}

absl::Time timestamp_get()
{
    return absl::FromUnixMicros(timestamp_get_us());
}
