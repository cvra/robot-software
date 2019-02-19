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
 *  Revision : $Id: obstacle_avoidance.c,v 1.1.2.8 2009-05-02 10:00:35 zer0 Exp $
 *
 *  Main code and algorithm: Fabrice DESCLAUX <serpilliere@droids-corp.org>
 *  Integration in Aversive: Olivier MATZ <zer0@droids-corp.org>
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <error/error.h>

#include <math/geometry/vect_base.h>
#include <math/geometry/lines.h>
#include <math/geometry/polygon.h>

#include <obstacle_avoidance/obstacle_avoidance.h>

#define GET_PT(a) (&(a) - &(oa->points[0]))

#define DEBUG_OA 0

#if DEBUG_OA == 1
#include "log.h"
#define DEBUG_OA_PRINTF(a, ...) log_message(a, ##__VA_ARGS__)
#else
#define DEBUG_OA_PRINTF(args...)
#endif

static void __oa_start_end_points(struct obstacle_avoidance* oa, int32_t st_x, int32_t st_y, int32_t en_x, int32_t en_y);

/* reset oa without reseting points coord */
static void oa_reset(struct obstacle_avoidance* oa)
{
    DEBUG_OA_PRINTF("%s()\r", __FUNCTION__);

    memset(oa->valid, 0, sizeof(oa->valid));
    memset(oa->pweight, 0, sizeof(oa->pweight));
    memset(oa->weight, 0, sizeof(oa->weight));
    memset(oa->p, 0, sizeof(oa->p));
    memset(oa->pt, 0, sizeof(oa->pt));
    memset(oa->rays, 0, sizeof(oa->rays));
    memset(oa->res, 0, sizeof(oa->res));
    oa->ray_n = 0;
    oa->res_len = 0;
}

/** Init the oa structure. Note: In the algorithm, the first polygon
 * is a dummy one, and is used to represent the START and END points
 * (so it has 2 vertices) */
void oa_init(struct obstacle_avoidance* oa)
{
    DEBUG_OA_PRINTF("%s()\r", __FUNCTION__);
    memset(oa, 0, sizeof(struct obstacle_avoidance));

    /* set a default start and point, reserve the first poly and
     * the first 2 points for it */
    oa->polys[0].pts = oa->points;
    oa->polys[0].l = 2;
    __oa_start_end_points(oa, 0, 0, 100, 100);
    oa->cur_pt_idx = 2;
    oa->cur_poly_idx = 1;
}

void oa_copy(struct obstacle_avoidance* dst, const struct obstacle_avoidance* oa)
{
    memset(dst, 0, sizeof(struct obstacle_avoidance));
    memcpy(dst, &oa, sizeof(struct obstacle_avoidance));
}

/**
 * Set the start and destination point. Return 0 on sucess
 */
static void __oa_start_end_points(struct obstacle_avoidance* oa, int32_t st_x, int32_t st_y, int32_t en_x, int32_t en_y)
{
    /* we always use the first 2 points of the table for start and end */
    oa->points[0].x = en_x;
    oa->points[0].y = en_y;

    /* Each point processed by Dijkstra is marked as valid. If we
     * have unreachable points (out of playground or points inside
     * polygons) Disjkstra won't mark them as valid. At the end of
     * the algorithm, if the destination point is not marked as
     * valid, there's no valid path to reach it. */

    oa->valid[GET_PT(oa->points[0])] = 0;
    /* the real dest is the start point for the algorithm */
    oa->pweight[GET_PT(oa->points[0])] = 1;

    oa->points[1].x = st_x;
    oa->points[1].y = st_y;
    oa->valid[GET_PT(oa->points[1])] = 0;
    oa->pweight[GET_PT(oa->points[1])] = 0;
}

/**
 * Set the start and destination point. Return 0 on sucess
 */
void oa_start_end_points(struct obstacle_avoidance* oa, int32_t st_x, int32_t st_y, int32_t en_x, int32_t en_y)
{
    DEBUG_OA_PRINTF("%s() (%ld,%ld) (%ld,%ld)\r", __FUNCTION__, st_x, st_y, en_x, en_y);
    __oa_start_end_points(oa, st_x, st_y, en_x, en_y);
}

/**
 * Create a new obstacle polygon. Return NULL on error.
 */
poly_t* oa_new_poly(struct obstacle_avoidance* oa, int size)
{
    DEBUG_OA_PRINTF("%s(size=%d)\r", __FUNCTION__, size);

    if (oa->cur_pt_idx + size > MAX_PTS) {
        return NULL;
    }
    if (oa->cur_poly_idx + 1 > MAX_POLY) {
        return NULL;
    }

    oa->polys[oa->cur_poly_idx].l = size;
    oa->polys[oa->cur_poly_idx].pts = &oa->points[oa->cur_pt_idx];
    oa->cur_pt_idx += size;

    return &oa->polys[oa->cur_poly_idx++];
}

int oa_segment_intersect_obstacle(struct obstacle_avoidance* oa, point_t p1, point_t p2)
{
    int i;
    point_t dummy;
    for (i = 0; i < oa->cur_poly_idx; i++) {
        if (is_crossing_poly(p1, p2, &dummy, &(oa->polys[i]))) {
            return 1;
        }
    }
    return 0;
}

/**
 * Add a point to the polygon.
 */
void oa_poly_set_point(struct obstacle_avoidance* oa,
                       poly_t* pol,
                       int32_t x,
                       int32_t y,
                       int i)
{
    DEBUG_OA_PRINTF("%s() (%ld,%ld)\r", "oa_s_p", x, y);

    pol->pts[i].x = x;
    pol->pts[i].y = y;
    oa->valid[GET_PT(pol->pts[i])] = 0;
    oa->pweight[GET_PT(pol->pts[i])] = 0;
}

int oa_get_path(struct obstacle_avoidance* oa, point_t** path)
{
    *path = oa->res;
    return oa->res_len;
}

void oa_dump(struct obstacle_avoidance* oa)
{
#if DEBUG_OA == 1
    int i, j;
    poly_t* poly;
    point_t* pt;

    DEBUG_OA_PRINTF("-- OA dump --\r");
    DEBUG_OA_PRINTF("nb_polys: %d\r", oa->cur_poly_idx);
    DEBUG_OA_PRINTF("nb_pts: %d\r", oa->cur_pt_idx);
    for (i = 0; i < oa->cur_poly_idx; i++) {
        poly = &oa->polys[i];
        DEBUG_OA_PRINTF("poly #%d\r", i);
        for (j = 0; j < poly->l; j++) {
            pt = &poly->pts[j];
            DEBUG_OA_PRINTF("  pt #%d (%2.0f,%2.0f)\r", j, pt->x, pt->y);
        }
    }
#else
    (void)oa;
#endif
}

/* Iterative Dijkstra algorithm: The valid filed is used to determine if:
 *   1: this point has been visited, his weight is correct.
 *   2: the point must be visited.
 *
 * The algorithm does: find a point that must be visited (2) update
 * his weight, mark it as (1) and update all his neightbours has 2.
 *
 * The algorithm ends when no (2) points are found
 *
 * A point with weight 0 is a point that has not been visited (so
 * weight is not affected yet); This explain why first point must have
 * a start weight different than 0.
 *
 * When the algo finds a shorter path to reach a point B from point A,
 * it will store in (p, pt) the parent point. This is important to
 * remenber and extract the solution path. */
void dijkstra(struct obstacle_avoidance* oa, int start_p, uint8_t start)
{
    int i;
    int8_t add;
    int8_t finish = 0;
    /* weight == 0 means not visited */
    /* weight == 1 for start */

    /* find all point linked to start */

    oa->valid[GET_PT(oa->polys[start_p].pts[start])] = 2;

    while (!finish) {
        finish = 1;

        for (start_p = 0; start_p < MAX_POLY; start_p++) {
            for (start = 0; start < oa->polys[start_p].l; start++) {
                if (oa->valid[GET_PT(oa->polys[start_p].pts[start])] != 2) {
                    continue;
                }
                add = -2;

                /* For all points that must be
                 * visited, we look for rays that
                 * start or stop at this point.  As
                 * ray array is (poly_num, point_num)
                 * we wtep 2 by 2. */
                for (i = 0; i < oa->ray_n; i += 2) {
                    /* If index is even in the
                     * aray, we have a start point
                     * ray, so connected point is
                     * i+2;
                     *
                     * If index is odd, we are in stop
                     * point and ray start point is at
                     * i-2 pos */
                    add = -add;

                    if (start_p != oa->rays[i] || start != oa->rays[i + 1]) {
                        continue;
                    }

                    if ((oa->pweight[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])] != 0) && (oa->pweight[GET_PT(oa->polys[start_p].pts[start])] + oa->weight[i / 4] >= oa->pweight[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])])) {
                        continue;
                    }

                    oa->p[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])] = start_p;
                    oa->pt[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])] = start;
                    oa->valid[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])] = 2;
                    oa->pweight[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])] =
                        oa->pweight[GET_PT(oa->polys[start_p].pts[start])] + oa->weight[i / 4];

                    oa->valid[GET_PT(oa->polys[start_p].pts[start])] = 1;
                    finish = 0;
                    DEBUG_OA_PRINTF("%s() (%2.0f,%2.0f p=%ld) %d (%2.0f,%2.0f p=%ld)\r",
                                    __FUNCTION__,
                                    oa->polys[start_p].pts[start].x,
                                    oa->polys[start_p].pts[start].y,
                                    oa->pweight[GET_PT(oa->polys[start_p].pts[start])],

                                    oa->weight[i / 4],

                                    oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]].x,
                                    oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]].y,
                                    oa->pweight[GET_PT(oa->polys[oa->rays[i + add]].pts[oa->rays[i + add + 1]])]);
                }
            }
        }
    }
}

/* display the path */
int8_t get_path(struct obstacle_avoidance* oa, poly_t* polys)
{
    int p, pt, p1, pt1, i;

    p = 0;
    pt = 1;
    i = 0;

    /* forget the first point */

    while (!(p == 0 && pt == 0)) {
        if (i >= MAX_CHKPOINTS) {
            return -1;
        }

        if (oa->valid[GET_PT(polys[p].pts[pt])] == 0) {
            DEBUG_OA_PRINTF("invalid path!\r");
            return -2;
        }

        p1 = oa->p[GET_PT(polys[p].pts[pt])];
        pt1 = oa->pt[GET_PT(polys[p].pts[pt])];
        p = p1;
        pt = pt1;
        oa->res[i].x = polys[p].pts[pt].x;
        oa->res[i].y = polys[p].pts[pt].y;
        DEBUG_OA_PRINTF("result[%d]: %2.0f, %2.0f\r", i, oa->res[i].x, oa->res[i].y);
        i++;
    }

    return i;
}

int8_t
oa_process(struct obstacle_avoidance* oa)
{
    int ret;
    int i;

    oa_reset(oa);

    /* First we compute the visibility graph */
    ret = calc_rays(oa->polys, oa->cur_poly_idx, oa->rays);
    DEBUG_OA_PRINTF("%s: %d rays\r", __FUNCTION__, ret);

    DEBUG_OA_PRINTF("Ray list\r");
    for (i = 0; i < ret; i += 4) {
        DEBUG_OA_PRINTF("%d,%d -> %d,%d\r", oa->rays[i], oa->rays[i + 1], oa->rays[i + 2],
                        oa->rays[i + 3]);
    }

    /* Then we affect the rays lengths to their weights */
    calc_rays_weight(oa->polys, oa->cur_poly_idx,
                     oa->rays, ret, oa->weight);

    DEBUG_OA_PRINTF("Ray weights:\r");
    for (i = 0; i < ret; i += 4) {
        DEBUG_OA_PRINTF("%d,%d->%d,%d (%d)\r",
                        (int)oa->polys[oa->rays[i]].pts[oa->rays[i + 1]].x,
                        (int)oa->polys[oa->rays[i]].pts[oa->rays[i + 1]].y,
                        (int)oa->polys[oa->rays[i + 2]].pts[oa->rays[i + 3]].x,
                        (int)oa->polys[oa->rays[i + 2]].pts[oa->rays[i + 3]].y,
                        oa->weight[i / 4]);
    }

    /* We aplly dijkstra on the visibility graph from the start
     * point (point 0 of the polygon 0) */
    oa->ray_n = ret;
    DEBUG_OA_PRINTF("dijkstra ray_n = %d\r", ret);
    dijkstra(oa, 0, 0);

    /* As dijkstra sets the parent points in the resulting graph,
     * we can backtrack the solution path. */
    oa->res_len = get_path(oa, oa->polys);
    return oa->res_len;
}
