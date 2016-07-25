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

#include <aversive.h>

#include <stdint.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <vect_base.h>
#include <lines.h>

#define DEBUG 0

#if DEBUG == 1
#define debug_printf(args...) printf(args)
#else
#define debug_printf(args...)
#endif

/* return values :
 *  0 dont cross
 *  1 cross
 *  2 parallel crossing
 *
 *  p argument is the crossing point coordinates (dummy for 0 or 2
 *  result)
 */
uint8_t intersect_line(const line_t *l1, const line_t *l2, point_t *p)
{
	float tmp1, tmp2;

	debug_printf("l1:%2.2f,%2.2f,%2.2f l2:%2.2f,%2.2f,%2.2f\n",
		     l1->a, l1->b, l1->c, l2->a, l2->b, l2->c);
	/* if dummy lines */
	if ((l1->a == 0 && l1->b == 0) || (l2->a == 0 && l2->b == 0))
		return 0;

	if (l1->a == 0) {
		if (l2->a == 0) {
			if (l1->b*l2->c == l2->b*l1->c)
				return 2;
			return 0;
		}

		/*       by  + c  = 0
		 * a'x + b'y + c' = 0 */
		/*
		  p->y = -l1->c/l1->b;
		  p->x = -(l2->b*p->y + l2->c)/l2->a;
		*/
		p->y = -l1->c/l1->b;
		p->x = -(l2->b*(-l1->c) + l2->c*l1->b)/(l2->a*l1->b);
		return 1;
	}

	if (l1->b == 0) {
		if (l2->b == 0) {
			if (l1->a*l2->c == l2->a*l1->c)
				return 2;
			return 0;
		}
		/* ax        + c  = 0
		 * a'x + b'y + c' = 0 */

		/*
		  p->x = -l1->c/l1->a;
		  p->y = -(l2->a*p->x + l2->c)/l2->b;
		*/
		p->x = -l1->c/l1->a;
		p->y = -(l2->a*(-l1->c) + l2->c*(l1->a))/(l2->b*l1->a);
		return 1;
	}

	/* parallel lines */
	if (l2->a*l1->b-l1->a*l2->b == 0) {
		if (l1->a*l2->c == l2->a*l1->c)
			return 2;
		return 0;
	}
	/*
	  p->y = (l1->a*l2->c - l1->c*l2->a)/(l2->a*l1->b - l1->a*l2->b);
	  p->x = -(l1->b*p->y+l1->c)/l1->a;
	*/
	tmp1 = (l1->a*l2->c - l1->c*l2->a);
	tmp2 = (l2->a*l1->b - l1->a*l2->b);
	p->y = tmp1 / tmp2;
	p->x = -(l1->b*tmp1 + l1->c*tmp2) / (l1->a*tmp2);
	return 1;
}

void pts2line(const point_t *p1, const point_t *p2, line_t *l)
{
	float p1x, p1y, p2x, p2y;

	p1x = p1->x;
	p1y = p1->y;
	p2x = p2->x;
	p2y = p2->y;


	l->a = -(p2y - p1y);
	l->b =  (p2x - p1x);
	l->c = -(l->a * p1x + l->b * p1y);

	debug_printf("%s: %2.2f, %2.2f, %2.2f\r\n",
		     __FUNCTION__, l->a, l->b, l->c);
}

void proj_pt_line(const point_t * p, const line_t * l, point_t * p_out)
{
	line_t l_tmp;

	l_tmp.a = l->b;
	l_tmp.b = -l->a;
	l_tmp.c = -l_tmp.a*p->x - l_tmp.b*p->y;

	p_out->y = (l_tmp.a*l->c - l->a*l_tmp.c) / (l->a*l_tmp.b - l_tmp.a*l->b);
	p_out->x = (l->b*l_tmp.c - l_tmp.b*l->c) / (l->a*l_tmp.b - l_tmp.a*l->b);

}



/* return values:
 *  0 dont cross
 *  1 cross
 *  2 cross on point
 *  3 parallel and one point in
 *
 *  p argument is the crossing point coordinates (dummy for 0 1 or 3
 *  result)
 */
uint8_t
intersect_segment(const point_t *s1, const point_t *s2,
		  const point_t *t1, const point_t *t2,
		  point_t *p)
{
	line_t l1, l2;
	uint8_t ret;
	int8_t u1, u2;
	vect_t v, w;

	debug_printf("s1:%"PRIi32",%"PRIi32" s2:%"PRIi32",%"PRIi32" "
		     "t1:%"PRIi32",%"PRIi32" t2:%"PRIi32",%"PRIi32"\r\n",
		     s1->x, s1->y, s2->x, s2->y,
		     t1->x, t1->y, t2->x, t2->y);

	pts2line(s1, s2, &l1);
	pts2line(t1, t2, &l2);

	ret = intersect_line(&l1, &l2, p);
	if (!ret)
		return 0;
	if (ret == 2) {
        	v.x = t1->x - s1->x;
        	v.y = t1->y - s1->y;
                w.x = t1->x - s2->x;
        	w.y = t1->y - s2->y;
		*p = *t1;
		if (vect_pscal_sign(&v, &w)<=0)
			return 3;

        	v.x = t2->x - s1->x;
        	v.y = t2->y - s1->y;
                w.x = t2->x - s2->x;
        	w.y = t2->y - s2->y;
		*p = *t2;
		if (vect_pscal_sign(&v, &w)<=0)
			return 3;
		return 0;
	}


	/* if points equal */
	if (s1->x == t1->x && s1->y == t1->y) {
		*p = *s1;
		return 2;
	}
	if (s1->x == t2->x && s1->y == t2->y) {
		*p = *s1;
		return 2;
	}
	if (s2->x == t1->x && s2->y == t1->y) {
		*p = *s2;
		return 2;
	}
	if (s2->x == t2->x && s2->y == t2->y) {
		*p = *s2;
		return 2;
	}

	debug_printf("px=%" PRIi32 " py=%" PRIi32 "\n", p->x, p->y);

	/* Consider as parallel if intersection is too far */
	if (ABS(p->x) > (1L << 15) || ABS(p->y) > (1L << 15))
		return 0;

	/* if prod scal neg: cut in middle of segment */
	v.x = p->x-s1->x;
	v.y = p->y-s1->y;
	w.x = p->x-s2->x;
	w.y = p->y-s2->y;
	u1 = vect_pscal_sign(&v, &w );

	v.x = p->x-t1->x;
	v.y = p->y-t1->y;
	w.x = p->x-t2->x;
	w.y = p->y-t2->y;
	u2 = vect_pscal_sign(&v, &w);

	debug_printf("u1=%d u2=%d\n", u1, u2);

	if (u1>0 || u2>0)
		return 0;

	if (u1==0 || u2==0)
		return 2;

	return 1;

}
void line_translate(line_t *l, vect_t *v)
{
	l->c -= (l->a * v->x + l->b * v->y);
}
