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
 *  Revision : $Id: position_manager.c,v 1.6.4.7 2009-05-18 12:27:26 zer0 Exp $
 *
 */

#include <string.h>
#include <math.h>

#include <aversive/position_manager/position_manager.h>
#include <aversive/robot_system/robot_system.h>

/** initialization of the robot_position pos, everthing is set to 0 */
void position_init(struct robot_position* pos)
{
    memset(pos, 0, sizeof(struct robot_position));
}

/** Set a new robot position */
void position_set(struct robot_position* pos, int16_t x, int16_t y, double a_deg)
{
    pos->pos_d.a = (a_deg * M_PI) / 180.0;
    pos->pos_d.x = x;
    pos->pos_d.y = y;
    pos->pos_s16.x = x;
    pos->pos_s16.y = y;
    pos->pos_s16.a = a_deg;
}

#ifdef CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
void position_set_centrifugal_coef(struct robot_position* pos, double coef)
{
    pos->centrifugal_coef = coef;
}
#endif

/**
 * Save in pos structure the pointer to the associated robot_system.
 * The robot_system structure is used to get values from virtual encoders
 * that return angle and distance.
 */
void position_set_related_robot_system(struct robot_position* pos, struct robot_system* rs)
{
    pos->rs = rs;
}

/**
 * Set the physical parameters of the robot :
 *  - number of impulsions for 1 mm (distance)
 *  - number of impulsions for 1 degree (angle)
 */
void position_set_physical_params(struct robot_position* pos, double track_mm, double distance_imp_per_mm)
{
    pos->phys.track_mm = track_mm;
    pos->phys.distance_imp_per_mm = distance_imp_per_mm;
}

void position_use_ext(struct robot_position* pos)
{
    struct rs_polar encoders;

    encoders.distance = rs_get_ext_distance(pos->rs);
    encoders.angle = rs_get_ext_angle(pos->rs);
    pos->prev_encoders = encoders;
    pos->use_ext = 1;
}

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
void position_use_mot(struct robot_position* pos)
{
    struct rs_polar encoders;

    encoders.distance = rs_get_mot_distance(pos->rs);
    encoders.angle = rs_get_mot_angle(pos->rs);
    pos->prev_encoders = encoders;
    pos->use_ext = 0;
}
#endif

/**
 * Process the absolute position (x,y,a) depending on the delta on
 * virtual encoders since last read, and depending on physical
 * parameters. The processed position is in mm.
 */
void position_manage(struct robot_position* pos)
{
    double x, y, a, r, arc_angle;
    double dx, dy;
    int16_t x_s16, y_s16, a_s16;
    struct rs_polar encoders;
    struct rs_polar delta;
    struct robot_system* rs;

    rs = pos->rs;
    /* here we could raise an error */
    if (rs == NULL) {
        return;
    }

#ifdef CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
    if (pos->use_ext) {
        encoders.distance = rs_get_ext_distance(rs);
        encoders.angle = rs_get_ext_angle(rs);
    } else {
        encoders.distance = rs_get_mot_distance(rs);
        encoders.angle = rs_get_mot_angle(rs);
    }
#else
    encoders.distance = rs_get_ext_distance(rs);
    encoders.angle = rs_get_ext_angle(rs);
#endif

    /* process difference between 2 measures.
     * No lock for prev_encoders since we are the only one to use
     * this var. */
    delta.distance = encoders.distance - pos->prev_encoders.distance;
    delta.angle = encoders.angle - pos->prev_encoders.angle;

    /* update double position */
    a = pos->pos_d.a;
    x = pos->pos_d.x;
    y = pos->pos_d.y;

    if (delta.angle == 0) {
        /* we go straight */
        dx = cos(a) * ((double)delta.distance / (pos->phys.distance_imp_per_mm));
        dy = sin(a) * ((double)delta.distance / (pos->phys.distance_imp_per_mm));
        x += dx;
        y += dy;
    } else {
        /* r the radius of the circle arc */
        r = (double)delta.distance * pos->phys.track_mm / ((double)delta.angle * 2);
        arc_angle = 2 * (double)delta.angle / (pos->phys.track_mm * pos->phys.distance_imp_per_mm);

        dx = r * (-sin(a) + sin(a + arc_angle));
        dy = r * (cos(a) - cos(a + arc_angle));

        x += dx;
        y += dy;
        a += arc_angle;

        if (a < -M_PI) {
            a += (M_PI * 2);
        } else if (a > (M_PI)) {
            a -= (M_PI * 2);
        }

#ifdef CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
        /* This part compensate the centrifugal force when we
        * turn very quickly. Idea is from Gargamel (RCVA). */
        if (pos->centrifugal_coef && r != 0) {
            double k;

            /*
             * centrifugal force is F = (m.v^2 / R)
             * with v: angular speed
             *      R: radius of the circle
             */

            k = ((double)delta.distance);
            k = k * k;
            k /= r;
            k *= pos->centrifugal_coef;

            /*
             * F acts perpendicularly to the vector
             */
            x += k * sin(a);
            y -= k * cos(a);
        }
#endif
    }

    pos->prev_encoders = encoders;

    /* update int position */
    x_s16 = (int16_t)x;
    y_s16 = (int16_t)y;
    a_s16 = (int16_t)(a * (360.0 / (M_PI * 2)));

    pos->pos_d.a = a;
    pos->pos_d.x = x;
    pos->pos_d.y = y;
    pos->pos_s16.x = x_s16;
    pos->pos_s16.y = y_s16;
    pos->pos_s16.a = a_s16;
}

/**
 * returns current x
 */
int16_t position_get_x_s16(struct robot_position* pos)
{
    return pos->pos_s16.x;
}

/**
 * returns current y
 */
int16_t position_get_y_s16(struct robot_position* pos)
{
    return pos->pos_s16.y;
}

/**
 * returns current alpha
 */
int16_t position_get_a_deg_s16(struct robot_position* pos)
{
    return pos->pos_s16.a;
}

/********* double */

/**
 * returns current x
 */
double position_get_x_double(struct robot_position* pos)
{
    return pos->pos_d.x;
}
float position_get_x_float(struct robot_position* pos)
{
    return (float)pos->pos_d.x;
}

/**
 * returns current y
 */
double position_get_y_double(struct robot_position* pos)
{
    return pos->pos_d.y;
}
float position_get_y_float(struct robot_position* pos)
{
    return (float)pos->pos_d.y;
}

vect2_cart position_get_xy_vect(struct robot_position* pos)
{
    vect2_cart r;
    r.x = (float)pos->pos_d.x;
    r.y = (float)pos->pos_d.y;
    return r;
}
/**
 * returns current alpha
 */
double position_get_a_rad_double(struct robot_position* pos)
{
    return pos->pos_d.a;
}

/**
 * returns current alpha
 */
float position_get_a_rad_float(struct robot_position* pos)
{
    return (float)(pos->pos_d.a);
}
