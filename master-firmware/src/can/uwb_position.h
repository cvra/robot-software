#ifndef UWB_POSITION_H
#define UWB_POSITION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <aversive/math/geometry/vect_base.h>
#include <timestamp/timestamp.h>

typedef struct {
    point_t point;
    timestamp_t timestamp;
} allied_position_t;

#ifdef __cplusplus
}
#endif

#endif /* UWB_POSITION_H */
