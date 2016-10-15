#ifndef MATH_HELPERS_H
#define MATH_HELPERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#define DEGREES(x) (x * 180.f / M_PI)
#define RADIANS(x) (x * M_PI / 180.f)

float angle_delta(float start, float end);


#ifdef __cplusplus
}
#endif

#endif /* MATH_HELPERS_H */
