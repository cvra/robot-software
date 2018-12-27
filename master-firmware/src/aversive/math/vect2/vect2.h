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
 *  Revision : $Id: vect2.h,v 1.3.4.1 2006-11-26 21:06:01 zer0 Exp $
 *
 */

/** \file vect2.h
 * \brief Interface for the 2 dimensional vector module.
 * \author JD & Vincent
 * \version 2.0
 * \date 21.12.2005 @ 23:09
 *
 * This module provides functions to handle 2d vectors and basic geometry fucntions.
 */

/** \addtogroup Vect2D
 * This module manages 2D vectors.
 * @{
 */

#ifndef _VECT2_H_
#define _VECT2_H_

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Cartesian vector structure
**/
typedef struct _vect2_cart {
    float x; /**< x-coordinate */
    float y; /**< y-coordinate */

} vect2_cart;

/** \brief Polar vector structure
**/
typedef struct _vect2_pol {
    float r; /**< Radius */
    float theta; /**< Angle */

} vect2_pol;

/************************ Begin prototyping ************************/

/** \brief Convert a polar vector to a cartesian one
 * \param vp reference to target polar vector to convert from.
 * \param vc reference to target cartesian vector where the function write.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 **/
void vect2_pol2cart(vect2_pol* vp, vect2_cart* vc);

/** \brief Convert a cartesian vector to a polar one
 * \param vc reference to target cartesian vector to convert from.
 * \param vp reference to target polar vector where the function write the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 **/
void vect2_cart2pol(vect2_cart* vc, vect2_pol* vp);

/** \brief Add 2 polar vectors and return the result
 * \param v1 Reference to a polar vector to sum.
 * \param v2 Reference to a polar vector to sum.
 * \param vresult Reference to a polar vector to store the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 * \note This function convert the 2 entry vectors to cartesian, sum them and then convert the result to polar.
 * So please think before using it.
 *
 * \f[ \vec V_{result} = \vec V_1 + \vec V_2 \f]
 **/
void vect2_add_pol(vect2_pol* v1, vect2_pol* v2, vect2_pol* vresult);

/** \brief Add 2 cartesian vectors and return the result
 * \param v1 Reference to a cartesian vector to sum.
 * \param v2 Reference to a cartesian vector to sum.
 * \param vresult Reference to a polar vector to store the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 * \f[ \vec V_{result} = \vec V_1 + \vec V_2 \f]
 **/
void vect2_add_cart(vect2_cart* v1, vect2_cart* v2, vect2_cart* vresult);

/** \brief Substract 2 polar vectors and return the result
 * \param v1 Reference to a polar vector to substract.
 * \param v2 Reference to a polar vector to substract.
 * \param vresult Reference to a polar vector to store the result.
 * \warning This function doesn't do any
    float R = 1.45064930529587234;
    float theta = 0.6734390282904231341;malloc ! You have to allocate structures before calling this function.
 * \note This function convert the 2 entry vectors to cartesian, substract them and then convert the result to polar.
 * So please think before using it.
 *
 * \f[ \vec V_{result} = \vec V_1 - \vec V_2 \f]
 **/
void vect2_sub_pol(vect2_pol* v1, vect2_pol* v2, vect2_pol* vresult);

/** \brief Substract 2 cartesian vectors and return the result
 * \param v1 Reference to a cartesian vector to substract.
 * \param v2 Reference to a cartesian vector to substract.
 * \param vresult Reference to a cartesian vector to store the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 * \f[ \vec V_{result} = \vec V_1 - \vec V_2 \f]
 **/
void vect2_sub_cart(vect2_cart* v1, vect2_cart* v2, vect2_cart* vresult);

/** \brief Multiply a cartesian vector by a scalar and return the result
 * \param v1 Reference to a cartesian vector.
 * \param alpha The multiplying scalar.
 * \param vresult Reference to a cartesian vector to store the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 * \f[ \vec V_{result} = \alpha\vec V_1 \f]
 **/
void vect2_scale_cart(vect2_cart* v1, float alpha, vect2_cart* vresult);

/** \brief Multiply a polar vector by a scalar and return the result
 * \param v1 Reference to a polar vector.
 * \param alpha The multiplying scalar.
 * \param vresult Reference to a cartesian vector to store the result.
 * \warning This function doesn't do any malloc ! You have to allocate structures before calling this function.
 *
 * \f[ \vec V_{result} = \alpha\vec V_1 \f]
 **/
void vect2_scale_pol(vect2_pol* v1, float alpha, vect2_pol* vresult);

/** @brief Norm of a cartesian vector.
 * @param vc Reference to a cartesian vector.
 **/
float vect2_norm_cart(vect2_cart* vc);

/** @brief Distance between two cartesian vectors.
 * @param vc1 Reference to a cartesian vector.
 * @param vc2 Reference to a cartesian vector.
 **/
float vect2_dist_cart(vect2_cart* vc1, vect2_cart* vc2);

/** @brief Inner product of two cartesian vectors.
 * @param vc1 Reference to a cartesian vector.
 * @param vc2 Reference to a cartesian vector.
 **/
float vect2_dot_product_cart(vect2_cart* vc1, vect2_cart* vc2);

/** @brief Angle between two cartesian vectors in radian.
 * @param vc1 Reference to a cartesian vector.
 * @param vc2 Reference to a cartesian vector.
 **/
float vect2_angle_2vec_rad_cart(vect2_cart* vc1, vect2_cart* vc2);

/** @brief Angle between vector and x-axis.
 * @param vc Reference to a cartesian vector.
 **/
float vect2_angle_vec_x_rad_cart(vect2_cart* vc);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /*_VECT2_H_*/
