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

#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#define DEBUG 0

#if DEBUG == 1
#define debug_printf(args...) printf(args)
#else
#define debug_printf(args...)
#endif

/* default bounding box is (0,0) (100,100) */
static int32_t bbox_x1 = 0;
static int32_t bbox_y1 = 0;
static int32_t bbox_x2 = 100;
static int32_t bbox_y2 = 100;


void polygon_set_boundingbox(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
	bbox_x1 = x1;
	bbox_y1 = y1;
	bbox_x2 = x2;
	bbox_y2 = y2;
}

uint8_t is_in_boundingbox(const point_t *p)
{
	if (p->x >= bbox_x1 &&
	    p->x <= bbox_x2 &&
	    p->y >= bbox_y1 &&
	    p->y <= bbox_y2)
		return 1;
	return 0;
}

/* Test if a point is in a polygon (including edges)
 *  0 not inside
 *  1 inside
 *  2 on edge
 */
uint8_t 
is_in_poly(const point_t *p, poly_t *pol)
{
	uint8_t i;
	uint8_t ii;
	int8_t z;
	uint8_t ret=1;
	vect_t v, w;

	for (i=0;i<pol->l;i++) {
		/* is a polygon point */
		if (p->x == pol->pts[i].x && p->y == pol->pts[i].y)
			return 2;
	}

	for (i=0;i<pol->l;i++) {

		ii = (i+1)%pol->l;
		v.x = pol->pts[ii].x-p->x;
		v.y = pol->pts[ii].y-p->y;
		w.x = pol->pts[i].x-p->x;
		w.y = pol->pts[i].y-p->y;
		z = vect_pvect_sign(&v, &w );
		if (z>0)
			return 0;
		if (z==0)
			ret=2;
	}

	return ret;
}

/* public wrapper for is_in_poly() */
uint8_t is_point_in_poly(poly_t *pol, int16_t x, int16_t y)
{
	point_t p;
	p.x = x;
	p.y = y;
	return is_in_poly(&p, pol);
}

/* Is segment crossing polygon? (including edges)
 *  0 don't cross
 *  1 cross
 *  2 on a side
 *  3 touch out (a segment boundary is on a polygon edge, 
 *  and the second segment boundary is out of the polygon)
 */
uint8_t 
is_crossing_poly(point_t p1, point_t p2, point_t *intersect_pt,
		 poly_t *pol)
{
	uint8_t i;
	uint8_t ret;
	point_t p;
	uint8_t ret1, ret2;
	uint8_t cpt=0;
	
	debug_printf("%" PRIi32 " %" PRIi32 " -> %" PRIi32 " %" PRIi32 " crossing poly %p ?\n", 
	       p1.x, p1.y, p2.x, p2.y, pol);
	debug_printf("poly is : ");
	for (i=0; i<pol->l; i++) {
		debug_printf("%" PRIi32 ",%" PRIi32 " ", pol->pts[i].x, pol->pts[i].y);
	}
	debug_printf("\n");

	for (i=0;i<pol->l;i++) {
		ret = intersect_segment(&p1, &p2, &pol->pts[i], &pol->pts[(i+1)%pol->l], &p);
		debug_printf("%" PRIi32 ",%" PRIi32 " -> %" PRIi32 ",%" PRIi32 
			     " return %d\n", pol->pts[i].x, pol->pts[i].y, 
		       pol->pts[(i+1)%pol->l].x, pol->pts[(i+1)%pol->l].y, ret);


		switch(ret) {
		case 0:
			break;
		case 1:
			if (intersect_pt)
				*intersect_pt = p;
			return 1;
			break;
		case 2:
			cpt++;
			if (intersect_pt)
				*intersect_pt = p;

			break;
		case 3:
			if (intersect_pt)
				*intersect_pt = p;
			return 2;
			break;
		}
	}

	if (cpt==3 ||cpt==4)
		return 1;

	ret1 = is_in_poly(&p1, pol);
	ret2 = is_in_poly(&p2, pol);

	debug_printf("is in poly: p1 %d p2: %d cpt %d\r\n", ret1, ret2, cpt);

	debug_printf("p intersect: %"PRIi32" %"PRIi32"\r\n", p.x, p.y);


	if (cpt==0) {
		if (ret1==1 || ret2==1)
			return 1;
		return 0;
	}


	if (cpt==1) {
		if (ret1==1 || ret2==1)
			return 1;
		return 3;
	}
	if (cpt==2) {
		if (ret1==1 || ret2==1)
			return 1;
		if (ret1==0 || ret2==0)
			return 3;
		return 1;
	}
	
	return 1;
}

/* Giving the list of poygons, compute the graph of "visibility rays".
 * This rays array is composed of indexes representing 2 polygon
 * vertices that can "see" each others:
 *
 *  i  : the first polygon number in the input polygon list
 *  i+1: the vertex index of this polygon (vertex 1)
 *  i+2: the second polygon number in the input polygon list
 *  i+3: the vertex index of this polygon (vertex 2)
 *
 *  Here, vertex 1 can "see" vertex 2 in our visibility graph
 *
 *  As the first polygon is not a real polygon but the start/stop
 *  point, the polygon is NOT an ocluding polygon (but its vertices
 *  are used to compute visibility to start/stop points)
 */

uint8_t 
calc_rays(poly_t *polys, uint8_t npolys, uint8_t *rays)
{
	uint8_t i, ii, index;
	uint8_t ray_n=0;
	uint8_t is_ok;
	uint8_t n;
	uint8_t pt1, pt2;

	/* !\\first poly is the start stop point */

	/* 1: calc inner polygon rays 
	 * compute for each polygon edges, if the vertices can see each others 
	 * (usefull if interlaced polygons)
	 */
	
	for (i=0; i<npolys; i++) {
		debug_printf("%s(): poly num %d/%d\n", __FUNCTION__, i, npolys);
		for (ii=0; ii<polys[i].l; ii++) {
			debug_printf("%s() line num %d/%d\n", __FUNCTION__, ii, polys[i].l);
			if (! is_in_boundingbox(&polys[i].pts[ii]))
				continue;
			is_ok = 1;
			n = (ii+1)%polys[i].l;

			if (!(is_in_boundingbox(&polys[i].pts[n])))
				continue;


			/* check if a polygon cross our ray */
			for (index=1; index<npolys; index++) {
				
				/* don't check polygon against itself */
				if (index == i) continue;
				
				if (is_crossing_poly(polys[i].pts[ii], polys[i].pts[n], NULL, 
						     &polys[index]) == 1) {
					is_ok = 0;
					debug_printf("is_crossing_poly() returned 1\n");
					break;
				}				    
			}
			/* if ray is not crossed, add it */
			if (is_ok) {
				rays[ray_n++] = i;
				rays[ray_n++] = ii;
				rays[ray_n++] = i;
				rays[ray_n++] = n;
			}
		}
	}


	/* 2: calc inter polygon rays.
	 * Visibility of inter-polygon vertices.*/

	/* For all poly */
	for (i=0; i<npolys-1; i++) {
		for (pt1=0;pt1<polys[i].l;pt1++) {

			if (!(is_in_boundingbox(&polys[i].pts[pt1])))
				continue;

			/* for next poly */
			for (ii=i+1; ii<npolys; ii++) {
				for (pt2=0;pt2<polys[ii].l;pt2++) {

					if (!(is_in_boundingbox(&polys[ii].pts[pt2])))
						continue;

					is_ok=1;
					/* test if a poly cross */
					for (index=1;index<npolys;index++) {
						if (is_crossing_poly(polys[i].pts[pt1], 
								     polys[ii].pts[pt2], NULL,
								     &polys[index]) == 1) {
							is_ok=0;
							break;
						}
					}
					/* if not crossed, we found a vilisity ray */
					if (is_ok) {
						rays[ray_n++] = i;
						rays[ray_n++] = pt1;
						rays[ray_n++] = ii;
						rays[ray_n++] = pt2;
					}			
				}
			}
		}	
	}
	
	
	return ray_n;
}

/* Compute the weight of every rays: the length of the rays is used
 * here. 
 *
 * Note the +1 is a little hack to introduce a preference between to
 * possiblity path: If we have 3 checpoint aligned in a path (say A,
 * B, C) the algorithm will prefer (A, C) instead of (A, B, C) */
void 
calc_rays_weight(poly_t *polys, __attribute__((unused)) uint8_t npolys,
		 uint8_t *rays, uint8_t ray_n, uint16_t *weight)
{
	uint8_t i;
	vect_t v;

	for (i=0;i<ray_n;i+=4) {
	        v.x = polys[rays[i]].pts[rays[i+1]].x - polys[rays[i+2]].pts[rays[i+3]].x;
	        v.y = polys[rays[i]].pts[rays[i+1]].y - polys[rays[i+2]].pts[rays[i+3]].y;
		weight[i/4] = vect_norm(&v) + 1;
	}	
}



