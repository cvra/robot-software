#ifndef POLAR_H
#define POLAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "encoder.h"

#define DEGREES(x) (x * 180.f / M_PI)
#define RADIANS(x) (x * M_PI / 180.f)

typedef struct {
    float distance;
    float angle;
} polar_t;

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar);
void polar_get_wheels_from_polar(const polar_t polar, wheels_t *wheels);

float angle_delta(float start, float end);



#ifdef __cplusplus
}
#endif

#endif /* POLAR_H */
