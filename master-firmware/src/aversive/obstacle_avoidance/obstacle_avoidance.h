/*
 *  Copyright Droids Corporation (2007)
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
 *  Revision : $Id: obstacle_avoidance.h,v 1.1.2.7 2009-05-02 10:00:35 zer0 Exp $
 *
 *  Main code and algorithm: Fabrice DESCLAUX <serpilliere@droids-corp.org>
 *  Integration in Aversive: Olivier MATZ <zer0@droids-corp.org>
 */

/* @file obstacle_avoidance.h
 *
 * @brief Implements static obstacle avoidance.
 *
 * The algorithm is based on the "visible point" algorithm.
 * There are 3 inputs:
 *   - the play ground (basically the table, here a rectangle)
 *   - the objects to avoid, represented by polygones
 *   - start/stop points (A, B)
 *
 * The algorithm will first find every ray formed by 2 points that can
 * "see" each others. Basically, if a polygon is between two points,
 * they cannot see each others. A side of a polygon is composed by 2
 * points that can se each others.
 *
 * From all these rays, we can create a graph. We affect for each ray
 * a weight with its own length.
 *
 * The algorithm executes Dijkstra to find the shortest path to go
 * from A to B.
 */

/*
 * As we run on 4Ko ram uC, we have static structures arrays to store:
 *  - MAX_POLY => represent the maximum polygons to avoid in the area.
 *  - MAX_PTS => maximize the sum of every polygons vertices.
 *  - MAX_RAYS => maximum number of rays.
 *  - MAX_CHKPOINTS => maximum accepted checkpoints in the resulting path.
 */

#ifndef _OBSTACLE_AVOIDANCE_H_
#define _OBSTACLE_AVOIDANCE_H_

#include <math/geometry/polygon.h>
#include <math/geometry/vect_base.h>
#include <math/geometry/lines.h>
#include <math/geometry/circles.h>

#define MAX_POLY 20        /**< The maximal number of obstacles in the area. */
#define MAX_PTS 500         /**< The maximal number of polygon vertices. */
#define MAX_RAYS 2000       /**< The maximal number of rays. */
#define MAX_CHKPOINTS 100   /**< Maximal length of the path. */


/** @struct obstacle_avoidance
 * @brief Instance of the obstacle avoidance system.
 *
 * This structure holds everything needed by the obstacle avoidance module,
 * like obstacles position, etc...
 *
 * To save memory space here is the memory representation of
 *   polygons/points:
 *\verbatim
    We have an array of points (oa_ext_point_t points):
   _____ _____ _____ _____ _____ _____ _____ _____ _____
 |     |     |     |     |     |     |     |     |     |
 | p0  | p1  | p0  | p1  | p2  | p3  | p0  | p1  | p2  |
 |_____|_____|_____|_____|_____|_____|_____|_____|_____|


   ^            ^                       ^
 |            |                       |
   -polygon 0   -polygon 1              -polygon 2
   -2 vertices  -4 vertices             -3 vertices
   \endverbatim
 *
 * And each polygon is represented by the sub array starting with the
 * point represented by oa_ext_point_t * pts and composed of uint8_t l;
 * (in the oa_poly_t structure)
 */
struct obstacle_avoidance {
    poly_t polys[MAX_POLY];  /**< Array of polygons (obstacles). */
    point_t points[MAX_PTS]; /**< Array of points, referenced by polys */
    uint8_t valid[MAX_PTS]; /**< Used by the Dijkstra algorithm to say if a point was visited. */
    int32_t pweight[MAX_PTS]; /**< Weight of a point in Dijkstra. */
    uint8_t p[MAX_PTS]; /**< @todo Dafuq ? */
    uint8_t pt[MAX_PTS]; /**< Stores all the points. */



    uint8_t ray_n; /**< Number of computed rays. */
    uint8_t cur_poly_idx; /**< Index of the current polygon (for adding polygons). */
    uint8_t cur_pt_idx; /**< Index of the current point in the current polygon. */

    uint16_t weight[MAX_RAYS]; /**< Length of each ray. */
    uint8_t rays[MAX_RAYS * 2]; /**< All valid rays given by Dijkstra. */
    point_t res[MAX_CHKPOINTS]; /**< Resulting path. */
    int res_len; /** Path length */
};

/** Reset obstacle avoidance without cleaning points */
void oa_reset(void);

/** Init the obstacle avoidance structure. */
void oa_init(void);

/** Copies the obstacle avoidance state */
void oa_copy(struct obstacle_avoidance *copy);

/** Set the start and destination point. */
void oa_start_end_points(int32_t st_x, int32_t st_y, int32_t en_x, int32_t en_y);

/** Create a new obstacle polygon.
 * @param [in] size Number of point in the polygon.
 * @return NULL on error.
 * @return Adress of the polygon if OK.
 */
poly_t *oa_new_poly(uint8_t size);


/** Dump status of the obstacle avoidance. */
void oa_dump(void);

/** Set a point of the polygon.
 * @param [in] pol The polygon who is being set.
 * @param [in] x,y The coordinates of the point, in mm.
 * @param [in] i The index of the point.
 */
void oa_poly_set_point(poly_t *pol, int32_t x, int32_t y, uint8_t i);


/** Processes the path.
 * @returns The number of points in the path on sucess
 * @returns An error code < 0 in case of failure.
 */
int8_t oa_process(void);

/** Gets the computed path.
 *
 * @returns An array of points, giving the path from start to end.
 */
int oa_get_path(point_t **path);


/** Checks if a segment is intersecting any obstacle.
 * @param [in] p1, p2 THe two points defining the segment.
 * @returns 1 if the segment intersects an obstacle, 0 otherwise.
 */
int oa_segment_intersect_obstacle(point_t p1, point_t p2);

#endif /* _OBSTACLE_AVOIDANCE_H_ */
