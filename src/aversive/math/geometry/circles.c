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

#include <math.h>

#include <aversive.h>

#include <vect_base.h>
#include <circles.h>
//#include "../fast_math/fast_math.h"

static inline float sq(float x)
{
	return x*x;
}

int pt_is_inside_circle(const point_t *p, circle_t *c)
{
	vect_t v;
	v.x = p->x - c->x;
	v.y = p->y - c->y;
	if ((v.x * v.x + v.y * v.y) < (c->r * c->r))
		return 1;
	return 0;
}

/*
 * return values:
 *  0 dont cross
 *  1 one intersection point
 *  2 two intersection points
 *
 *  p1, p2 arguments are the crossing points coordinates. Both p1 and
 *  p2 are dummy for 0 result. When result is 1, p1 and p2 are set to
 *  the same value.
 */
int circle_intersect(const circle_t *c1, const circle_t *c2,
			 point_t *p1, point_t *p2)
{
	circle_t ca, cb;
	float a, b, c, d, e;
	uint8_t ret = 0;


    /* We have to assume that either delta_x or delta_y is not zero to avoid
     * the corner case of the two coincident circles. If we assume that, for
     * example, delta_y is never zero, but in reality gets really close to zero,
     * it creates numerical stability problems. To avoid it, we check which of
     * delta_x or delta_y is smaller and assume the other cannot be zero.
     */
    if (fabs(c1->y - c2->y) < fabs(c1->x - c2->x)) {
        point_t pa, pb;
        ca.x = c1->y;
        ca.y = c1->x;
        ca.r = c1->r;
        cb.x = c2->y;
        cb.y = c2->x;
        cb.r = c2->r;
        ret = circle_intersect(&ca, &cb, &pa, &pb);
        p1->x=pa.y;
        p1->y=pa.x;
        p2->y=pb.x;
        p2->x=pb.y;
        return ret;
    }

	/* create circles with same radius, but centered on 0,0 : it
	 * will make process easier */
	ca.x = 0;
	ca.y = 0;
	ca.r = c1->r;
	cb.x = c2->x - c1->x;
	cb.y = c2->y - c1->y;
	cb.r = c2->r;

	/* inspired from http://www.loria.fr/~roegel/notes/note0001.pdf
	 * which can be found in doc. */
	a = 2.0f * cb.x;
	b = 2.0f * cb.y;
	c = sq(cb.x) + sq(cb.y) - sq(cb.r) + sq(ca.r);
	d = sq(2.0f * a * c) -
		(4.0f * (sq(a) + sq(b)) * (sq(c) - sq(b) * sq(ca.r)) );

	/* no intersection */
	if (d < 0.0f)
		return 0;


	if (fabsf(b) <  0.0001f) {
		/* special case */
		e = sq(cb.r) - sq((2.0f * c - sq(a)) / (2.0f * a));

		/* no intersection */
		if (e < 0.0f)
			return 0;

		p1->x = (2.0f * a * c - sqrtf(d)) / (2.0f * (sq(a) + sq(b)));
		p1->y = sqrtf(e);
		p2->x = p1->x;
		p2->y = p1->y;
		ret = 1;
	}
	else {
		/* usual case */
		p1->x = (2.0f * a * c - sqrtf(d)) / (2.0f * (sq(a) + sq(b)));
		p1->y = (c - a * p1->x) / b;
		p2->x = (2.0f * a * c + sqrtf(d)) / (2.0f * (sq(a) + sq(b)));
		p2->y = (c - a * p2->x) / b;
		ret = 2;
	}

	/* retranslate */
	p1->x += c1->x;
	p1->y += c1->y;
	p2->x += c1->x;
	p2->y += c1->y;

	return ret;
}
