/*
 *  Copyright Droids Corporation, Microb Technology, Eirbot (2005)
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
 *  Revision : $Id: quadramp.c,v 1.4.4.7 2009-05-18 12:29:51 zer0 Exp $
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "quadramp/quadramp.h"

void quadramp_init(struct quadramp_filter* q)
{
    memset(q, 0, sizeof(*q));
}

void quadramp_reset(struct quadramp_filter* q)
{
    q->previous_var = 0;
    q->previous_out = 0;
    q->previous_in = 0;
}

void quadramp_set_2nd_order_vars(struct quadramp_filter* q,
                                 double var_2nd_ord_pos,
                                 double var_2nd_ord_neg)
{
    q->var_2nd_ord_pos = var_2nd_ord_pos;
    q->var_2nd_ord_neg = var_2nd_ord_neg;
}

void quadramp_set_1st_order_vars(struct quadramp_filter* q,
                                 double var_1st_ord_pos,
                                 double var_1st_ord_neg)
{
    q->var_1st_ord_pos = var_1st_ord_pos;
    q->var_1st_ord_neg = var_1st_ord_neg;
}

void quadramp_set_position(struct quadramp_filter* q, int32_t pos)
{
    q->previous_out = pos;
    q->previous_var = 0;
}

uint8_t quadramp_is_finished(struct quadramp_filter* q)
{
    return (int32_t)q->previous_out == q->previous_in && q->previous_var == 0;
}

int32_t quadramp_do_filter(void* data, int32_t in)
{
    struct quadramp_filter* q = data;
    int32_t d; /* remaining distance */
    int32_t pos_target;
    double var_1st_ord_pos = q->var_1st_ord_pos;
    double var_1st_ord_neg = -q->var_1st_ord_neg;
    double var_2nd_ord_pos = q->var_2nd_ord_pos;
    double var_2nd_ord_neg = -q->var_2nd_ord_neg;
    double previous_var, d_float;
    double previous_out;

    previous_var = q->previous_var;
    previous_out = q->previous_out;

    d_float = (double)in - previous_out;

    /* d is very small, we can jump to dest */
    if (fabs(d_float) < 2.) {
        q->previous_var = 0;
        q->previous_out = in;
        q->previous_in = in;
        return in;
    }

    d = (int32_t)d_float;

    /* Deceleration ramp */
    if (d > 0 && var_2nd_ord_neg) {
        double ramp_pos;
        /* var_2nd_ord_neg < 0 */
        ramp_pos = sqrt((var_2nd_ord_neg * var_2nd_ord_neg) / 4 - 2 * d_float * var_2nd_ord_neg) + var_2nd_ord_neg / 2;

        if (ramp_pos < var_1st_ord_pos) {
            var_1st_ord_pos = ramp_pos;
        }
    } else if (d < 0 && var_2nd_ord_pos) {
        double ramp_neg;

        /* var_2nd_ord_pos > 0 */
        ramp_neg = -sqrt((var_2nd_ord_pos * var_2nd_ord_pos) / 4 - 2 * d_float * var_2nd_ord_pos) - var_2nd_ord_pos / 2;

        /* ramp_neg < 0 */
        if (ramp_neg > var_1st_ord_neg) {
            var_1st_ord_neg = ramp_neg;
        }
    }

    /* try to set the speed : can we reach the speed with our acceleration ? */
    /* si on va moins vite que la Vmax */
    if (previous_var < var_1st_ord_pos) {
        /* acceleration would be to high, we reduce the speed */
        /* si rampe acceleration active ET qu'on ne peut pas atteindre Vmax,
         * on sature Vmax a Vcourante + acceleration */
        if (var_2nd_ord_pos && (var_1st_ord_pos - previous_var) > var_2nd_ord_pos) {
            var_1st_ord_pos = previous_var + var_2nd_ord_pos;
        }
    }
    /* si on va plus vite que Vmax */
    else if (previous_var > var_1st_ord_pos) {
        /* deceleration would be to high, we increase the speed */
        /* si rampe deceleration active ET qu'on ne peut pas atteindre Vmax,
         * on sature Vmax a Vcourante + deceleration */
        if (var_2nd_ord_neg && (var_1st_ord_pos - previous_var) < var_2nd_ord_neg) {
            var_1st_ord_pos = previous_var + var_2nd_ord_neg;
        }
    }

    /* same for the neg */
    /* si on va plus vite que la Vmin (en negatif : en vrai la vitesse absolue est inferieure) */
    if (previous_var > var_1st_ord_neg) {
        /* acceleration would be to high, we reduce the speed */
        /* si rampe deceleration active ET qu'on ne peut pas atteindre Vmin,
         * on sature Vmax a Vcourante + deceleration */
        if (var_2nd_ord_neg && (var_1st_ord_neg - previous_var) < var_2nd_ord_neg) {
            var_1st_ord_neg = previous_var + var_2nd_ord_neg;
        }
    }
    /* si on va moins vite que Vmin (mais vitesse absolue superieure) */
    else if (previous_var < var_1st_ord_neg) {
        /* deceleration would be to high, we increase the speed */
        /* si rampe acceleration active ET qu'on ne peut pas atteindre Vmin,
         * on sature Vmax a Vcourante + deceleration */
        if (var_2nd_ord_pos && (var_1st_ord_neg - previous_var) > var_2nd_ord_pos) {
            var_1st_ord_neg = previous_var + var_2nd_ord_pos;
        }
    }

    /*
     * Position consign : can we reach the position with our speed ?
     */
    if (d_float > var_1st_ord_pos) {
        pos_target = previous_out + var_1st_ord_pos;
        previous_var = var_1st_ord_pos;
    } else if (d_float < var_1st_ord_neg) {
        pos_target = previous_out + var_1st_ord_neg;
        previous_var = var_1st_ord_neg;
    } else {
        pos_target = in;
        previous_var = d_float;
    }

    // update previous_out and previous_var
    q->previous_var = previous_var;
    q->previous_out = pos_target;
    q->previous_in = in;

    return pos_target;
}
