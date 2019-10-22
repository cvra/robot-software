#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

/* Timestamp
 * monotonic, provides us resolution, overflows every 1.19 hours
 */
typedef uint32_t timestamp_t;

/* Long Timestamp
 * monotonic, provides us resolution, never overflows (every 593066 years)
 */
typedef uint64_t ltimestamp_t;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Obtain current timestamp
 */
timestamp_t timestamp_get(void);
ltimestamp_t ltimestamp_get(void);

/*
 * Compute duration between two timestamps
 *  The duration from the first timestamp (t1) to the second (t2) can be
 *  positive or negative. It is positive when t1 was taken before t2.
 * Overflow handling:
 *  Overflows of the internal integer representation are correctly handled.
 *  To distinguish between positive and negative durations, the shorter of the
 *  possible intervals is chosen.
 */
int32_t timestamp_duration_us(timestamp_t t1, timestamp_t t2);
float timestamp_duration_s(timestamp_t t1, timestamp_t t2);

int64_t ltimestamp_duration_us(ltimestamp_t t1, ltimestamp_t t2);
float ltimestamp_duration_s(ltimestamp_t t1, ltimestamp_t t2);

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */
