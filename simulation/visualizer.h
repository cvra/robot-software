#ifndef VISUALIZER_H
#define VISUALIZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "math/geometry/polygon.h"
#include "math/geometry/vect_base.h"

#ifdef __cplusplus
}
#endif

/* Non thread safe setters */
void visualizer_set_path(point_t start, point_t* new_path, size_t new_len);
void visualizer_set_obstacles(poly_t* new_obstacles, size_t new_len);

void visualizer_run(void);


#endif /* VISUALIZER_H */
