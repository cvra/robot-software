#ifndef POLAR_H
#define POLAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "encoder.h"

typedef struct {
    float distance;
    float angle;
} polar_t;

void polar_get_polar_from_wheels(const wheels_t wheels, polar_t *polar);


#ifdef __cplusplus
}
#endif

#endif /* POLAR_H */
