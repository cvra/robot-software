/*
 *  Copyright Droids Corporation (2009)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: f16.h,v 1.6.4.3 2008-05-10 15:06:26 zer0 Exp $
 *
 */

#ifndef _CIRCLES_H_
#define _CIRCLES_H_

#include <aversive/math/geometry/vect_base.h>

/** \addtogroup Geometrie
 * This module does some simple calculation on circles, lines, and polygons.
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Coordinates and radius of a circle */
typedef struct _circle {
    float x; /**< x-coordinate */
    float y; /**< y-coordinate */
    float r; /**< radius */
} circle_t;

/** Checks if a point is inside a circle.
 * @param [in] *p Point
 * @param [in] *c Circle
 * @return 1 if p is inside c, 0 otherwise.
 */
int pt_is_inside_circle(const point_t* p, circle_t* c);

/** Checks if 2 circles are intersecting.
 * @return The number of intersection points (0, 1 or 2)
 * @param [in] *c1 The first circle.
 * @param [in] *c2 The second circle.
 * @param [out] *p1 The first intersection point.
 * @param [out] *p2 The second intersection point.
 *
 * @note If the 2 circles intersect on 1 point, then p1 == p2.
 */
int circle_intersect(const circle_t* c1, const circle_t* c2, point_t* p1, point_t* p2);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _CIRCLES_H_ */
