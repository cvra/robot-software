#pragma once

#include <absl/time/time.h>

/** Returns the number of microseconds since UNIX epoch. Guaranteed to be
 * monotoonic (never goes back in time), means that it can be different from
 * the computer's local time. */
int64_t timestamp_get_us();

/** Returns an absl::Time representation of the time represented in
 * timestamp_get_us(). This is different from absl::Now(), which returns a
 * non-monotonic clock. */
absl::Time timestamp_get();
