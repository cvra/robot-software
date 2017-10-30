#ifndef LIE_GROUPS_H
#define LIE_GROUPS_H

#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/vect_base.h>

// Special Orientation group 2 (SO2) defines a 2D rotation
typedef struct {
    float angle;
} so2_t;

so2_t so2_create(float angle);

point_t so2_rotate(so2_t rotation, point_t point);

#endif
