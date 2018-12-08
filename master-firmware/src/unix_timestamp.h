#ifndef UNIX_TIMESTAMP_H
#define UNIX_TIMESTAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Timestamp in UNIX format. */
typedef struct {
    int32_t s; /**< Seconds since UNIX epoch. */
    int32_t us; /**< microseconds since last second. */
} unix_timestamp_t;

/** @brief Converts a UNIX timestamp to microseconds since boot.
 *
 * @warning This function will return bogus timestamps if no reference point
 * was set using timestamp_set_refence().
 */
int32_t timestamp_unix_to_local_us(unix_timestamp_t ts);

/** @brief Converts a local time in microseconds since boot to a UNIX timestamp.
 *
 * @warning This function will return bogus timestamps if no reference point
 * was set using timestamp_set_refence().
 */
unix_timestamp_t timestamp_local_us_to_unix(int32_t ts);

/** Sets a reference point for synchronization. */
void timestamp_set_reference(unix_timestamp_t unix_ts, int32_t local_ts);

/** Compares two UNIX timestamps.
 *
 * @return -1 If a < b
 * @return 0 If a == b
 * @return 1 If a > b
 *
 * @note The return value convention is the same as used for libc's qsort
 * function.
 */
int timestamp_unix_compare(unix_timestamp_t a, unix_timestamp_t b);

#ifdef __cplusplus
}
#endif

#endif
