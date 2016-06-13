#ifndef POLAR_H
#define POLAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"

#define DEGREES(x) (x * 180 / M_PI)
#define RADIANS(x) (x * M_PI / 180)

typedef struct {
    float distance;
    float angle;
} polar_t;

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar);

float angle_delta(float start, float end);



#ifdef __cplusplus
}
#endif

#endif /* POLAR_H */
