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
 *  Revision : $Id: position_manager.h,v 1.5.4.4 2009-05-18 12:27:26 zer0 Exp $
 *
 */

#ifndef _ROBOT_POSITION_MANAGER_H_
#define _ROBOT_POSITION_MANAGER_H_

#include <stdint.h>
#include <math.h>

#include <aversive/math/vect2/vect2.h>

#include <aversive/robot_system/robot_system.h>

/** \addtogroup Odometry
 * This module manages the encoders to compute the robot's position.
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enables centrifugal force compensation.
 *
 * A vehicle moving on a trajectory is subject to the centrifugal force F. This
 * force pulls the vehicle inside his own trajectory, which causes position
 * error who standard odometry systems do not account for.
 * This software includes a method to compensaite such errors. The main idea
 * is that since F is orthogonal to the speed vector, we suppose that the
 * lateral displacement will be pureley proportionnal to this force.
 *
 * The centrifugal force can be written as
 *
 * \f$\begin{array}{lcl}
 * F &=&\frac{v^2}{R}\cdot m \\
 *   &=& M \cdot \omega^2 \cdot R\\
 *   &=& M \cdot V \cdot \omega\end{array}\f$
 *
 * The third form is the most interseting one because it does not require
 * the computation of the radius R. At any given point n of the trajectory
 * we can easily compute
 *
 * \f$\begin{array}{ĺcl}
 * V_n &=& D_n - D_{n-1}\\
 * \omega_n &=& \theta_n - \theta_{n-1}
 * \end{array}\f$
 *
 * Lets write :
 *
 * \f$\begin{array}{lcl}
 * \Delta X &=& V_n \cos\theta_n\\
 * \Delta Y &=& V_n \sin\theta_n
 * \end{array} \f$
 *
 * In a classic odometry implementation, the equations of odometry are :
 *
 * \f$\begin{array}{lcl}
 * X_{n+1} &=& X_n + \Delta X \\
 * Y_{n+1} &=& Y_n + \Delta Y
 * \end{array} \f$
 *
 * In the modified implementation we have :
 *
 * \f$\begin{array}{lccccc}
 * X_{n+1} &=& X_n &+& \Delta X  &-& k \omega_n\ \Delta Y\\
 * Y_{n+1} &=& Y_n &+& \Delta Y  &+& k \omega_n\ \Delta X
 * \end{array} \f$
 *
 * K is found by experience, and given to the system using
 * position_set_centrifugal_coef().
 *
 * \todo For now the code does not use the algorithm written above but another
 * equivalent, slower implementation.
 * */
#define CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE

/** @brief Stores the physical dimensions of the robot.
 *
 * This structure stores the number of encoder impulsion per mm and the
 * distance between the wheels of the robot.
 */
struct robot_physical_params {
    double track_mm; /**< Track (distance between wheels) in mm */
    double distance_imp_per_mm; /**< Impulsions per mm */
};

/** @brief Stores a cartesian position in double.
 *
 * This structure holds a position of the robot in the double precision format.
 * @sa xya_position_s16
 */
struct xya_position {
    double x; /**< The X coordinate, in mm. */
    double y; /**< The Y coordinate, in mm. */
    double a; /**< The angle relative to the X axis, in radians. */
};

/**@brief Stores a cartesian position in int.
 *
 * Same as xya_position but in integers.
 * @note The angle is stocked in degrees.
 * @sa xya_position
 */
struct xya_position_s16 {
    int16_t x; /**< The X coordinate, in mm. */
    int16_t y; /**< The Y coordinate, in mm. */
    int16_t a; /**< The angle relative to the X axis in degrees. */
};

/** \brief Instance of the odometry subsystem.
 *
 * This structure holds everything that is needed to compute and store the
 * position of the robot.
 */
struct robot_position {
    uint8_t use_ext; /**< Only useful when we have 2 sets of encoders. */
    struct robot_physical_params phys; /**< The physical parameters of the robot. */
    struct xya_position pos_d; /**< Position of the robot in double. */
    struct xya_position_s16 pos_s16; /**< Position of the robot in integers. */
    struct rs_polar prev_encoders; /**< Previous state of the encoders. */
    struct robot_system* rs; /**< Robot system used for the computations. */

#ifdef CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
    double centrifugal_coef; /**< Coefficient for the centrifugal computation */
#endif
};

/** @brief Initialization of the odometry subsystem.
 *
 * This function initialize a robot_position structure, setting everything to 0.
 * @param [in] pos The odometry instance to initialize.
 */
void position_init(struct robot_position* pos);

#ifdef CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
/** @brief Set centrifugal coef.
 *
 * This functions sets the centrifugal coef for the odometry compensation.
 * @param [in] pos The odometry instance.
 * @param [in] coef The centrifugal force coefficient.
 * @sa CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
 */
void position_set_centrifugal_coef(struct robot_position* pos, double coef);
#endif

/** @brief Set a new robot position.
 * @param [in] pos The odometry instance.
 * @param [in] x, y The new coordinate of the robot, in mm.
 * @param [in] a_deg The new angle of the robot, in degree.
 */
void position_set(struct robot_position* pos, int16_t x, int16_t y, double a_deg);

/** @brief Tells the robot to use the separate wheels encoders.
 *
 * If the robot has input for both motor encoders and separate wheel encoders,
 * the user can instruct the robot to use one or another encoder depending on
 * the situation.
 * @param [in] pos The odometry instance.
 * @note By default the odometry system uses the separate wheel encoders.
 */
void position_use_ext(struct robot_position* pos);

/** @brief Tells the robot to use the motor encoders.
 *
 * @param [in] pos The odometry instance.
 * @sa position_use_ext */
void position_use_mot(struct robot_position* pos);

/** @brief Sets the physical parameters of the robot.
 * @param [in] pos The robot_position instance to configure.
 * @param [in] track_mm The distance between the wheels, in mm.
 * @param [in] distance_imp_per_mm The number of encoder pulses for one mm.
 */
void position_set_physical_params(struct robot_position* pos, double track_mm, double distance_imp_per_mm);

/** @brief Set related robot_system structure.
 *
 * Save in pos structure the pointer to the associated robot_system.
 * The robot_system structure is used to get values from virtual encoders
 * that return angle and distance.
 *
 * @param [in] pos The odometry system instance.
 * @param [in] rs The robot_system instance.
 */
void position_set_related_robot_system(struct robot_position* pos, struct robot_system* rs);

/** @brief Updates the position.
 *
 * Process the absolute position (x,y,a) depending on the delta on
 * virtual encoders since last read, and depending on physical
 * parameters.
 *
 * @param [in] pos The odometry system instance.
 *
 * @note This function should be called at a fixed interval to ensure good
 * results.
 */
void position_manage(struct robot_position* pos);

/** @brief Get current X.
 *
 * @param [in] pos The odometry system instance.
 * @return Current x in mm in integer.
 */
int16_t position_get_x_s16(struct robot_position* pos);

/** @brief Get current Y.
 *
 * @param [in] pos The odometry system instance.
 * @return Current Y in mm in integer.
 */
int16_t position_get_y_s16(struct robot_position* pos);

/** @brief Get current angle.
 *
 * @param [in] pos The odometry system instance.
 * @return Current angle in degrees in integer.
 */
int16_t position_get_a_deg_s16(struct robot_position* pos);

/** @brief Get current X.
 *
 * @param [in] pos The odometry system instance.
 * @return Current x in mm in double.
 */
double position_get_x_double(struct robot_position* pos);

/** @brief Get current X.
 *
 * @param [in] pos The odometry system instance.
 * @return Current x in mm in float.
 */
float position_get_x_float(struct robot_position* pos);

/** @brief Get current Y.
 *
 * @param [in] pos The odometry system instance.
 * @return Current Y in mm in double.
 */
double position_get_y_double(struct robot_position* pos);

/** @brief Get current Y.
 *
 * @param [in] pos The odometry system instance.
 * @return Current Y in mm in float.
 */
float position_get_y_float(struct robot_position* pos);

/** @brief Get current position
 *
 * @param [in] pos The odometry system instance.
 * @returns current position stored in a vect2_cart.
 */
vect2_cart position_get_xy_vect(struct robot_position* pos);

/** @brief Returns current angle.
 *
 * @param [in] pos The odometry system instance.
 * @returns Current angle in radians in double.
 */
double position_get_a_rad_double(struct robot_position* pos);

/** @brief Returns current angle.
 *
 * @param [in] pos The odometry system instance.
 * @returns Current angle in radians in float.
 */
float position_get_a_rad_float(struct robot_position* pos);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
