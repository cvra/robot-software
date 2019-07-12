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
 *  Revision : $Id: angle_distance.h,v 1.3.4.3 2007-06-17 21:23:41 zer0 Exp $
 *
 */

#ifndef _ANGLE_DISTANCE_H_
#define _ANGLE_DISTANCE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Stores the state of two encoders or pwm in the left / right format. */
struct rs_wheels {
    int32_t left; /**< The left encoder value. */
    int32_t right; /**< The right encoder value. */
};

/** Stores the state of two encoders or pwm in the distance / angle format. */
struct rs_polar {
    int32_t distance; /**< The distance encoder value. */
    int32_t angle; /**< The angle encoder value. */
};

/** @brief Converts the state of two encoders or pwm.
 *
 * This function converts the left / right representation to a angle distance
 * representation.
 *
 * @param [in] w_src The source state, in left / right format.
 * @param [out] p_dst The destination state, in distance / angle format.
 */
void rs_get_polar_from_wheels(struct rs_polar* p_dst, struct rs_wheels* w_src);

/** @brief Converts the state of two encoders or pwm.
 *
 * This function converts the angle / distance representation to an left-right representation.
 *
 * @param [in] p_src The source state in distance / angle format.
 * @param [out] The destination state in left / right format.
 */
void rs_get_wheels_from_polar(struct rs_wheels* w_dst, struct rs_polar* p_src);

#ifdef __cplusplus
}
#endif

#endif
