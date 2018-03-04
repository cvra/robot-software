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
point_t so2_inverse_rotate(so2_t rotation, point_t point);

// Translation
vect_t translation_2d(float x, float y);

// Special Euclidian group 2 (SE2) defines a 2D rigid body transform (rotation + translation)
typedef struct {
    so2_t rotation;
    vect_t translation;
} se2_t;

se2_t se2_create(float angle, vect_t translation);
se2_t se2_create_xya(float x, float y, float angle);

point_t se2_transform(se2_t transform, point_t point);
point_t se2_inverse_transform(se2_t transform, point_t point);

// Chains the transforms: res = lhs @ rhs
se2_t se2_chain(se2_t lhs, se2_t rhs);

// Returns the inverse of input transform
se2_t se2_inverse(se2_t transform);

#endif
