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
 *  Revision : $Id: angle_distance.c,v 1.4.4.4 2009-03-05 22:02:55 zer0 Exp $
 *
 */

#include "aversive/robot_system/angle_distance.h"

/**
 * convert the values of wheels encoders (left, right) into (distance,
 * angle)
 */
void rs_get_polar_from_wheels(struct rs_polar* p_dst, struct rs_wheels* w_src)
{
    p_dst->distance = (w_src->right + w_src->left) / 2;
    p_dst->angle = (w_src->right - w_src->left) / 2;
}

/**
 * convert (distance, angle) into (left, right)
 */
void rs_get_wheels_from_polar(struct rs_wheels* w_dst, struct rs_polar* p_src)
{
    w_dst->left = p_src->distance - p_src->angle;
    w_dst->right = p_src->distance + p_src->angle;
}
