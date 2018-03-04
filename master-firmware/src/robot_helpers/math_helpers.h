#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include "math/geometry/polygon.h"

#define DEGREES(x) ((x) * 180.f / M_PI)
#define RADIANS(x) ((x) * M_PI / 180.f)

/** Return the minimum angular distance between the start and end angle
 */
float angle_delta(float start, float end);

/** Check if the given point is inside the given square
 * @note polygon needs points to be ordered counter clock wise
 */
bool math_point_is_in_square(poly_t* square, int x, int y);

/** Clamp the value to fit in the specified interval
 */
int math_clamp_value(int value, int min, int max);


#ifdef __cplusplus
}
#endif

#endif /* MATH_HELPERS_H */
