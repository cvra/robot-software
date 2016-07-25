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
#include <math.h>
#include <vect_base.h>
//#include "../fast_math/fast_math.h"

/* Return scalar product */
float
vect_pscal(vect_t *v, vect_t *w)
{
	return v->x * w->x + v->y * w->y;
}

/* Return Z of vectorial product */
float
vect_pvect(vect_t *v, vect_t *w)
{
	return v->x*w->y - v->y*w->x;
}

/* Return sign of scalar product */
int8_t
vect_pscal_sign(vect_t *v, vect_t *w)
{
	float z;
	z = vect_pscal(v, w);
	if (z==0)
		return 0;
	return z>0?1:-1;
}

/* Return sign of vectorial product */
int8_t
vect_pvect_sign(vect_t *v, vect_t *w)
{
	float z;
	z = vect_pvect(v, w);
	if (z==0)
		return 0;
	return z>0?1:-1;
}

float xy_norm(float x1, float y1, float x2, float y2)
{
	float x = x2 - x1;
	float y = y2 - y1;
	return sqrtf(x*x + y*y);
}

float pt_norm(const point_t *p1, const point_t *p2)
{
	float x = p2->x - p1->x;
	float y = p2->y - p1->y;
	return sqrtf(x*x + y*y);
}

/* norm of a vector */
float vect_norm(const vect_t *v)
{
	return sqrtf(v->x*v->x+v->y*v->y);
}

void vect_rot_trigo(vect_t *v)
{
	float s;

	s = v->x;
	v->x= -v->y;
	v->y = s;
}

void vect_rot_retro(vect_t *v)
{
	float s;

	s = v->x;
	v->x= v->y;
	v->y = -s;
}


float vect_get_angle(vect_t *v, vect_t *w)
{
	float ps;
	float n;

	ps = vect_pscal(v, w);
	n = vect_norm(v) * vect_norm(w);

	return acosf((float)ps/n);
}

void vect_resize(vect_t *v, float l)
{
	float old_l = vect_norm(v);
	float x = v->x, y = v->y;
	v->x = x * l / old_l;
	v->y = y * l / old_l;
}
