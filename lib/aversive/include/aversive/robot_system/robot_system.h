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
 *  Revision : $Id: robot_system.h,v 1.5.4.3 2008-04-06 17:33:57 zer0 Exp $
 *
 */

/** @file robot_system.h
 *
 * @brief Conversion from left / right encoders to angle / distance.
 *
 * This module is responsible for converting the values from left / right
 * wheels to virtual motor and encoders for angle and distance. This step
 * is sometimes called mixing in other implementations.
 *
 * To calculate the value for the virtual encoders, we simply set angle
 * = right-left and distance = left + right. There is not unit manipulation
 * at this level, everything is done in encoder ticks.
 *
 * To mix the value sent to the motor, we do something like left = distance +
 * angle and right = distance - angle.
 *
 * This modules also provides the ability to configure different gains
 * on left and right wheels, allowing for compensation of small differences
 * on coder wheel size.
 *
 * A small difference in wheel diameter can have a large effect on final
 * position error:
 * Lets write \f$D_1\f$ and \f$D_2\f$ the diameters of the two coding wheels.
 * We suppose that the robot is asked to go on a straight line and that the
 * control system manages to have no angular speed difference between the two
 * wheels. Then the difference in speed can be written as :
 *
 * \f$\begin{array}{ĺcl}
 * \frac{\Delta V}{V} &=& \frac{V_1-V_2}{V}\\
 *                   &=& \frac{\Delta D}{D}
 * \end{array}\f$
 *
 * We can easily show that the robot will not do a line, but instead a circle
 * whose radius is given by :
 *
 * \f$
 * R=\frac{A}{\frac{\Delta V}{V}}
 * \f$
 * Where A is the track (distance between wheels) of the robot.
 *
 * If we do a numerical application with typical data for a Eurobot robot :
 * - Track of 30cm
 * - \f$\frac{\Delta D}{D} = \frac{1}{1000}\f$
 *
 * The difference of diameter can seem very small, but if we do a straight
 * line of x=3 meters we will have a deviation in angle of
 * \f$\theta = \frac{1}{100} rd\f$
 *
 * The lateral displacement will be :
 * \f$\Delta Y = 2 \theta X = 0.06\f$
 *
 * This shows the necessity to compensate for the difference of diameter on
 * coding wheels as the robot will be 6cm off after a single table crossing !
 */

#include <stdint.h>
#include "angle_distance.h"

#ifndef _ROBOT_SYSTEM_H_
#define _ROBOT_SYSTEM_H_

/* flags */
/** This flags tells the module to use the external encoders. */
#define RS_USE_EXT 1

/** This flags tells the module to ignore the external coder gain. */
#define RS_IGNORE_EXT_GAIN 2

/** This flags tells the module to ignore the motor coder gain. */
#define RS_IGNORE_MOT_GAIN 4

#define CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT

/** @brief An instance of the robot_system module
 *
 * This structure holds everything we need in order to perform the left / right
 * to distance and angle conversion.
 *
 * @todo Remove the references to f64 and use floats instead.
 */
struct robot_system {
    uint8_t flags; /**< Flags currently in use. */

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    /* Motor encoders. */
    struct rs_polar pmot_prev; /**< Previous position of the virtual motor encoders. */
    struct rs_wheels wmot_prev; /**< Previous position of the real motor encoders. */

    float ratio_mot_ext; /**< Ratio between the motor and external encoders track. */

    /** Callback to get the value of the left motor encoder.*/
    int32_t (*left_mot_encoder)(void*);

    /** Data that will be passed to left_mot_encoder. */
    void* left_mot_encoder_param;

    /** Gain on the left motor encoder to compensate for wheel difference. */
    float left_mot_gain;

    /** Callback to get the value of the left motor encoder.*/
    int32_t (*right_mot_encoder)(void*);

    /** Data that will be passed to right_mot_encoder. */
    void* right_mot_encoder_param;

    /** Gain on the right motor encoder to compensate for wheel difference. */
    float right_mot_gain;
#endif

    /* External encoders */
    /** Value of the virtual PWM. */
    struct rs_polar virtual_pwm;

    /** Value of the virtual encoders. */
    struct rs_polar virtual_encoders;

    /** Previous value of the virtual encoders. */
    struct rs_polar pext_prev;

    /** Previous value of the real encoders. */
    struct rs_wheels wext_prev;

    /** Callback to get the value of the left encoder. */
    int32_t (*left_ext_encoder)(void*);

    /** Data that will be passed to left_ext_encoder. */
    void* left_ext_encoder_param;

    /** Gain on the left encoder to compensate for wheel difference. */
    double left_ext_gain;

    /** Callback to get the value of the right encoder. */
    int32_t (*right_ext_encoder)(void*);

    /** Data that will be passed to right_ext_encoder. */
    void* right_ext_encoder_param;

    /** Gain on the right encoder to compensate for wheel difference. */
    double right_ext_gain;

    /* PWM */

    /** Callback to set the left motor PWM. */
    void (*left_pwm)(void*, int32_t);

    /** Data that will be passed to left_pwm. */
    void* left_pwm_param;

    /** Callback to set the right motor PWM. */
    void (*right_pwm)(void*, int32_t);

    /** Data that will be passed to right_pwm. */
    void* right_pwm_param;
};

/** @brief Inits the instance.
 *
 * Sets all the fields to 0.
 * @param [in] rs The robot_system instance.
 */
void rs_init(struct robot_system* rs);

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
/** @brief Sets the ratio between ext and mot.
 *
 * Sets the ratio between the motor wheels track and the external encoder
 * track
 *
 * @param [in] rs The robot_system instance.
 * @param [in] ratio The ratio mot_track / ext_track.
 */
void rs_set_ratio(struct robot_system* rs, double ratio);
#endif

/** @brief Define left pwn.
 *
 * This function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *left_pwm The function that will be stored as left PWM callback.
 * @param [in] left_pwm_param The data that will be passed to left_pwm.
 */
void rs_set_left_pwm(struct robot_system* rs, void (*left_pwm)(void*, int32_t), void* left_pwm_param);

/** @brief Define right pwn.
 *
 * This function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *right_pwm The function that will be stored as right PWM callback.
 * @param [in] right_pwm_param The data that will be passed to right_pwm.
 */
void rs_set_right_pwm(struct robot_system* rs, void (*right_pwm)(void*, int32_t), void* right_pwm_param);

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
/** @brief Define left motor encoder.
 *
 * THis function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *left_mot_encoder The function that will be used as left pwm callback.
 * @param [in] left_mot_encoder_param The data that will be passed to left_mot_encoder.
 * @param gain The gain that will be used for diameter compensation.
 */
void rs_set_left_mot_encoder(struct robot_system* rs, int32_t (*left_mot_encoder)(void*), void* left_mot_encoder_param, double gain);

/** @brief Define right motor encoder.
 *
 * THis function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *right_mot_encoder The function that will be used as right pwm callback.
 * @param [in] right_mot_encoder_param The data that will be passed to right_mot_encoder.
 * @param gain The gain that will be used for diameter compensation.
 */
void rs_set_right_mot_encoder(struct robot_system* rs, int32_t (*right_mot_encoder)(void*), void* right_mot_encoder_param, double gain);
#endif

/** @brief Define left external encoder.
 *
 * THis function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *left_ext_encoder The function that will be used as left pwm callback.
 * @param [in] left_ext_encoder_param The data that will be passed to left_ext_encoder.
 * @param gain The gain that will be used for diameter compensation.
 */
void rs_set_left_ext_encoder(struct robot_system* rs, int32_t (*left_ext_encoder)(void*), void* left_ext_encoder_param, double gain);

/** @brief Define right external encoder.
 *
 * THis function tells robot_system which callback to use.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] *right_ext_encoder The function that will be used as right pwm callback.
 * @param [in] right_ext_encoder_param The data that will be passed to right_ext_encoder.
 * @param gain The gain that will be used for diameter compensation.
 */
void rs_set_right_ext_encoder(struct robot_system* rs, int32_t (*right_ext_encoder)(void*), void* right_ext_encoder_param, double gain);

/** @brief Sets the angle PWM.
 *
 * This function sets the real PWMs according to the existing distance PWM and
 * the given angle PWM.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] angle The value of the angle PWM.
 */
void rs_set_angle(void* rs, int32_t angle);

/** @brief Sets the distance PWM.
 *
 * This function sets the real PWMs according to the existing angle PWM and
 * the given distance PWM.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] distance The value of the angle PWM.
 */
void rs_set_distance(void* rs, int32_t distance);

/** @brief Get the angle encoder value.
 *
 * This function returns the computed angle encoder value based on real encoders
 * value and gains.
 *
 * This value is computed as angle=(right-left)/2, which means positive angle is
 * counter clockwise.
 *
 * @param [in] rs The robot_system instance.
 * @return The computed value for angle.
 */
int32_t rs_get_angle(void* rs);

/** @brief Get the distance encoder value.
 *
 * This function returns the computed distance encoder value based on real encoders
 * value and gains.
 *
 * This value is computed as distance=(right+left)/2, which means positive angle is
 * counter clockwise.
 *
 * @param [in] rs The robot_system instance.
 * @return The computed value for distance.
 */
int32_t rs_get_distance(void* rs);

/** @brief Gets the angle based on external encoders.
 *
 * This function does the same as rs_get_angle() but forces usage of external
 * encoders.
 *
 * @sa rs_get_angle
 * @param [in] rs The robot_system instance.
 * @returns The angle value according to external encoders.
 */
int32_t rs_get_ext_angle(void* rs);

/** @brief Gets the distance based on external encoders.
 *
 * This function does the same as rs_get_distance() but forces usage of external
 * encoders.
 *
 * @sa rs_get_distance
 * @param [in] rs The robot_system instance.
 * @returns The angle distance according to external encoders.
 */
int32_t rs_get_ext_distance(void* rs);

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT

/** @brief Gets the angle based on motor encoders.
 *
 * This function does the same as rs_get_angle() but forces usage of motor
 * encoders.
 *
 * @sa rs_get_angle
 * @sa rs_get_ext_angle
 * @param [in] rs The robot_system instance.
 * @returns The angle value according to motor encoders.
 */
int32_t rs_get_mot_angle(void* rs);

/** @brief Gets the distance based on motor encoders.
 *
 * This function does the same as rs_get_distance() but forces usage of motor
 * encoders.
 *
 * @sa rs_get_distance
 * @sa rs_get_ext_distance
 * @param [in] rs The robot_system instance.
 * @returns The distance according to motro encoders.
 */
int32_t rs_get_mot_distance(void* rs);
#endif

/* same for left/right */
/** @brief Gets left external encoder value
 *
 * @param [in] rs The robot_system instance.
 *
 * @return The left external encoder value.
 */
int32_t rs_get_ext_left(void* rs);

/** @brief Gets right external encoder value
 *
 * @param [in] rs The robot_system instance.
 *
 * @return The right external encoder value.
 */
int32_t rs_get_ext_right(void* rs);

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT

/** @brief Gets left motor encoder value
 *
 * @param [in] rs The robot_system instance.
 *
 * @return The left motor encoder value.
 */
int32_t rs_get_mot_left(void* rs);

/** @brief Gets right motor encoder value
 *
 * @param [in] rs The robot_system instance.
 *
 * @return The right motor encoder value.
 */
int32_t rs_get_mot_right(void* rs);
#endif

/** @brief Update the encoder values.
 *
 * Read the encoders, and update internal virtual counters. Call this
 * function is needed before reading the virtual encoders. The program
 * will decide if it the external encoders or the motor encoders are
 * taken in account.
 *
 * @param [in] rs The robot_system instance.
 */
void rs_update(void* rs);

/** @brief Sets the flags.
 *
 * @param [in] rs The robot_system instance.
 * @param [in] flags A set of OR'ed flags telling which encoder to use and how
 * to apply gain.
 *
 * @sa RS_USE_EXT
 * @sa RS_IGNORE_EXT_GAIN
 * @sa RS_IGNORE_MOT_GAIN
 */
void rs_set_flags(struct robot_system* rs, uint8_t flags);

#endif /* #ifndef _ROBOT_SYSTEM */
