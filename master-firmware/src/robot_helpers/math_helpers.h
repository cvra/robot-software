#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include "math/geometry/polygon.h"

#define DEGREES(x) (x * 180.f / M_PI)
#define RADIANS(x) (x * M_PI / 180.f)

float angle_delta(float start, float end);

bool math_point_is_in_square(poly_t* square, int x, int y);


#ifdef __cplusplus
}
#endif

#endif /* MATH_HELPERS_H */
