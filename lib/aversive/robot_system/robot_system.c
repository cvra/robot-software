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
 *  Revision : $Id: robot_system.c,v 1.6.4.7 2009-03-29 18:48:23 zer0 Exp $
 *
 */

/**
 * The goal of this module is to provide an interface to motor and
 * encoders of the robot. This module provide a function that returns
 * the value of virtual encoders containing distance and angle. It
 * also allow the user to command virtual angle and distance PWMs.
 */

#include <string.h>

#include "aversive/robot_system/angle_distance.h"
#include "aversive/robot_system/robot_system.h"

/** @brief Call a pwm pointer :
 *
 * This function reads the pointer to the PWM function and if the pointer is not
 * null call the set PWM function.
 *
 * @param [in] rs The robot_system instance.
 */
static inline void safe_setpwm(void (*f)(void*, int32_t), void* param, int32_t value)
{
    void (*f_tmp)(void*, int32_t);
    void* param_tmp;
    f_tmp = f;
    param_tmp = param;
    if (f_tmp) {
        f_tmp(param_tmp, value);
    }
}

/** @brief Call an encoder pointer :
 *
 * This function reads the pointer to the encoder function and if the pointer
 * is not null call the encoder function and return its result. If it is null,
 * then return 0.
 *
 * @param [in] rs The robot_system instance.
 *
 * @return The value of the encoder if defined, 0 otherwise.
 */
static inline uint32_t safe_getencoder(int32_t (*f)(void*), void* param)
{
    int32_t (*f_tmp)(void*);
    void* param_tmp;
    f_tmp = f;
    param_tmp = param;
    if (f_tmp) {
        return f_tmp(param_tmp);
    }
    return 0;
}

void rs_init(struct robot_system* rs)
{
    memset(rs, 0, sizeof(struct robot_system));
#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    rs_set_ratio(rs, 1.0);
#endif
}

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
void rs_set_ratio(struct robot_system* rs, double ratio)
{
    rs->ratio_mot_ext = ratio;
}
#endif

void rs_set_left_pwm(struct robot_system* rs, void (*left_pwm)(void*, int32_t), void* left_pwm_param)
{
    rs->left_pwm = left_pwm;
    rs->left_pwm_param = left_pwm_param;
}

void rs_set_right_pwm(struct robot_system* rs, void (*right_pwm)(void*, int32_t), void* right_pwm_param)
{
    rs->right_pwm = right_pwm;
    rs->right_pwm_param = right_pwm_param;
}

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
void rs_set_left_mot_encoder(struct robot_system* rs, int32_t (*left_mot_encoder)(void*), void* left_mot_encoder_param, double gain)
{
    rs->left_mot_encoder = left_mot_encoder;
    rs->left_mot_encoder_param = left_mot_encoder_param;
    rs->left_mot_gain = gain;
}

void rs_set_right_mot_encoder(struct robot_system* rs, int32_t (*right_mot_encoder)(void*), void* right_mot_encoder_param, double gain)
{
    rs->right_mot_encoder = right_mot_encoder;
    rs->right_mot_encoder_param = right_mot_encoder_param;
    rs->right_mot_gain = gain;
}
#endif

void rs_set_left_ext_encoder(struct robot_system* rs, int32_t (*left_ext_encoder)(void*), void* left_ext_encoder_param, double gain)
{
    rs->left_ext_encoder = left_ext_encoder;
    rs->left_ext_encoder_param = left_ext_encoder_param;
    rs->left_ext_gain = gain;
}

void rs_set_right_ext_encoder(struct robot_system* rs, int32_t (*right_ext_encoder)(void*), void* right_ext_encoder_param, double gain)
{
    rs->right_ext_encoder = right_ext_encoder;
    rs->right_ext_encoder_param = right_ext_encoder_param;
    rs->right_ext_gain = gain;
}

void rs_set_angle(void* data, int32_t angle)
{
    struct rs_polar p;
    struct rs_wheels w;
    struct robot_system* rs = data;
    p.distance = rs->virtual_pwm.distance;
    rs->virtual_pwm.angle = angle;

    p.angle = angle;
    rs_get_wheels_from_polar(&w, &p);

    safe_setpwm(rs->left_pwm, rs->left_pwm_param, w.left);
    safe_setpwm(rs->right_pwm, rs->right_pwm_param, w.right);
}

void rs_set_distance(void* data, int32_t distance)
{
    struct robot_system* rs = data;
    struct rs_polar p;
    struct rs_wheels w;

    p.angle = rs->virtual_pwm.angle;
    rs->virtual_pwm.distance = distance;

    p.distance = distance;
    rs_get_wheels_from_polar(&w, &p);

    safe_setpwm(rs->left_pwm, rs->left_pwm_param, w.left);
    safe_setpwm(rs->right_pwm, rs->right_pwm_param, w.right);
}

int32_t rs_get_angle(void* data)
{
    struct robot_system* rs = data;
    return rs->virtual_encoders.angle;
}

int32_t rs_get_distance(void* data)
{
    struct robot_system* rs = data;
    return rs->virtual_encoders.distance;
}

int32_t rs_get_ext_angle(void* data)
{
    struct robot_system* rs = data;
    return rs->pext_prev.angle;
}

int32_t rs_get_ext_distance(void* data)
{
    struct robot_system* rs = data;
    return rs->pext_prev.distance;
}

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
int32_t rs_get_mot_angle(void* data)
{
    struct robot_system* rs = data;
    return rs->pmot_prev.angle;
}

int32_t rs_get_mot_distance(void* data)
{
    struct robot_system* rs = data;

    return rs->pmot_prev.distance;
}
#endif

int32_t rs_get_ext_left(void* data)
{
    struct robot_system* rs = data;
    return rs->wext_prev.left;
}

int32_t rs_get_ext_right(void* data)
{
    struct robot_system* rs = data;
    return rs->wext_prev.right;
}

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
int32_t rs_get_mot_left(void* data)
{
    struct robot_system* rs = data;
    return rs->wmot_prev.left;
}

int32_t rs_get_mot_right(void* data)
{
    struct robot_system* rs = data;
    return rs->wmot_prev.right;
}
#endif

void rs_set_flags(struct robot_system* rs, uint8_t flags)
{
    rs->flags = flags;
}

void rs_update(void* data)
{
    struct robot_system* rs = data;
    struct rs_wheels wext;
    struct rs_polar pext;
#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    struct rs_wheels wmot;
    struct rs_polar pmot;
#endif
    int32_t delta_angle, delta_distance;

    /* read encoders */
    wext.left = safe_getencoder(rs->left_ext_encoder, rs->left_ext_encoder_param);
    wext.right = safe_getencoder(rs->right_ext_encoder, rs->right_ext_encoder_param);

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    wmot.left = safe_getencoder(rs->left_mot_encoder, rs->left_mot_encoder_param);
    wmot.right = safe_getencoder(rs->right_mot_encoder, rs->right_mot_encoder_param);
#endif

    /* apply gains to each wheel */
    if (!(rs->flags & RS_IGNORE_EXT_GAIN)) {
        double tmp;
        tmp = wext.left;
        tmp *= rs->left_ext_gain;
        wext.left = tmp;
        tmp = wext.right;
        tmp *= rs->right_ext_gain;
        wext.right = tmp;
    }

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    if (!(rs->flags & RS_IGNORE_MOT_GAIN)) {
        wmot.left = wmot.left * rs->left_mot_gain;
        wmot.right = wmot.right * rs->right_mot_gain;
    }
#endif

    rs_get_polar_from_wheels(&pext, &wext);
#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    rs_get_polar_from_wheels(&pmot, &wmot);

    /* apply ratio to polar and reupdate wheels for ext coders */
    pext.angle = pext.angle * rs->ratio_mot_ext;
    rs_get_wheels_from_polar(&wext, &pext);
#endif

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    /* update from external encoders */
    if (rs->flags & RS_USE_EXT) {
        delta_angle = pext.angle - rs->pext_prev.angle;
        delta_distance = pext.distance - rs->pext_prev.distance;
    }
    /* update from motor encoders */
    else {
        delta_angle = pmot.angle - rs->pmot_prev.angle;
        delta_distance = pmot.distance - rs->pmot_prev.distance;
    }
#else
    delta_angle = pext.angle - rs->pext_prev.angle;
    delta_distance = pext.distance - rs->pext_prev.distance;
#endif

    rs->virtual_encoders.angle += delta_angle;
    rs->virtual_encoders.distance += delta_distance;
    rs->pext_prev = pext;
    rs->wext_prev = wext;

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    rs->pmot_prev = pmot;
    rs->wmot_prev = wmot;
#endif
}
