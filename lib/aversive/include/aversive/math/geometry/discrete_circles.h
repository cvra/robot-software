#ifndef AVERSIVE_MATH_GEOMETRY_DISCRETE_CIRCLES_H
#define AVERSIVE_MATH_GEOMETRY_DISCRETE_CIRCLES_H

#include <stdbool.h>

#include <aversive/math/geometry/circles.h>
#include <aversive/math/geometry/polygon.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Computes polygon inscribed in given circle.
 * The first point of the polygon starts at angle offset.
 *
 * @param poly[in,out] Polygon that contains discrete circle
 * @param circle[in] Circle in which the polygon is inscribed
 * @param samples[in] Number of samples of the discrete circle (i.e. number of polygon vertices)
 * @param angle_offset[in] Starting angle in radians
 *
 * @return true if successfully computed discrete circle, false otherwise
 */
bool discretize_circle(poly_t* poly, circle_t circle, int samples, float angle_offset);

#ifdef __cplusplus
}
#endif

#endif /* AVERSIVE_MATH_GEOMETRY_DISCRETE_CIRCLES_H */
