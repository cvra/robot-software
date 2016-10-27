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
 *  Revision : $Id: trajectory_manager.c,v 1.4.4.17 2009-05-18 12:28:36 zer0 Exp $
 *
 */

/* Trajectory Manager v3 - zer0 - for Eurobot 2010 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "math/vect2/vect2.h"
#include "math/geometry/vect_base.h"
#include "math/geometry/lines.h"
#include "error/error.h"

#include "position_manager/position_manager.h"
#include "robot_system/robot_system.h"
#include "control_system_manager/control_system_manager.h"
#include "quadramp/quadramp.h"

#include <trajectory_manager/trajectory_manager.h>
#include "trajectory_manager/trajectory_manager_utils.h"
#include "trajectory_manager/trajectory_manager_core.h"

/************ SIMPLE TRAJS, NO EVENT */

#define UPDATE_D 1
#define UPDATE_A 2
#define RESET_D  4
#define RESET_A  8


static uint8_t evt_debug_cpt = 0;
#define EVT_DEBUG(args ...) do {             \
        if (((evt_debug_cpt ++) & 0x07) == 0) { \
            DEBUG(args);            \
        }                   \
} while (0)

static void start_clitoid(struct trajectory *traj);

/** @brief Update angle and/or distance
 *
 * this function is not called directly by the user
 * @param [in] traj Pointer to the trajectory structure.
 * @param [in] d_mm Distance in mm.
 * @param [in] a_rad Angle in radians.
 * @param [in] flags What to update (UPDATE_A, UPDATE_D)
 */
void __trajectory_goto_d_a_rel(struct trajectory *traj, double d_mm,
                               double a_rad, uint8_t state, uint8_t flags)
{
    int32_t a_consign, d_consign;

    DEBUG(E_TRAJECTORY, "Goto DA/RS rel to d=%f a_rad=%f", d_mm, a_rad);
    delete_event(traj);
    traj->state = state;
    if (flags & UPDATE_A) {
        if (flags & RESET_A) {
            a_consign = 0;
        } else {
            a_consign = (int32_t)(a_rad * (traj->position->phys.distance_imp_per_mm) *
                                  (traj->position->phys.track_mm) / 2);
        }
        a_consign +=  rs_get_angle(traj->robot);
        traj->target.pol.angle = a_consign;
        cs_set_consign(traj->csm_angle, a_consign);
    }
    if (flags & UPDATE_D) {
        if (flags & RESET_D) {
            d_consign = 0;
        } else {
            d_consign = (int32_t)((d_mm) * (traj->position->phys.distance_imp_per_mm));
        }
        d_consign += rs_get_distance(traj->robot);
        traj->target.pol.distance = d_consign;
        // printf("D set %f\n", (float)d_consign);
        cs_set_consign(traj->csm_distance, d_consign);
    }
}

void trajectory_d_rel(struct trajectory *traj, double d_mm)
{
    __trajectory_goto_d_a_rel(traj, d_mm, 0, RUNNING_D,
                              UPDATE_D | UPDATE_A | RESET_A);
}

void trajectory_only_d_rel(struct trajectory *traj, double d_mm)
{
    __trajectory_goto_d_a_rel(traj, d_mm, 0, RUNNING_D, UPDATE_D);
}

void trajectory_a_rel(struct trajectory *traj, double a_deg_rel)
{
    // printf("%d\n", (int)a_deg_rel);
    __trajectory_goto_d_a_rel(traj, 0, RAD(a_deg_rel), RUNNING_A,
                              UPDATE_A | UPDATE_D | RESET_D);
}

void trajectory_a_abs(struct trajectory *traj, double a_deg_abs)
{
    double posa = position_get_a_rad_double(traj->position);
    double a;

    a = RAD(a_deg_abs) - posa;
    a = modulo_2pi(a);
    __trajectory_goto_d_a_rel(traj, 0, a, RUNNING_A,
                              UPDATE_A | UPDATE_D | RESET_D);
}

void trajectory_turnto_xy(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
    double posx = position_get_x_double(traj->position);
    double posy = position_get_y_double(traj->position);
    double posa = position_get_a_rad_double(traj->position);

    DEBUG(E_TRAJECTORY, "Goto Turn To xy %f %f", x_abs_mm, y_abs_mm);
    __trajectory_goto_d_a_rel(traj, 0,
                              simple_modulo_2pi(atan2(y_abs_mm - posy, x_abs_mm - posx) - posa),
                              RUNNING_A,
                              UPDATE_A | UPDATE_D | RESET_D);
}

void trajectory_turnto_xy_behind(struct trajectory *traj, double x_abs_mm, double y_abs_mm)
{
    double posx = position_get_x_double(traj->position);
    double posy = position_get_y_double(traj->position);
    double posa = position_get_a_rad_double(traj->position);

    DEBUG(E_TRAJECTORY, "Goto Turn To xy %f %f", x_abs_mm, y_abs_mm);
    __trajectory_goto_d_a_rel(traj, 0,
                              modulo_2pi(atan2(y_abs_mm - posy, x_abs_mm - posx) - posa + M_PI),
                              RUNNING_A,
                              UPDATE_A | UPDATE_D | RESET_D);
}

void trajectory_only_a_rel(struct trajectory *traj, double a_deg)
{
    __trajectory_goto_d_a_rel(traj, 0, RAD(a_deg), RUNNING_A,
                              UPDATE_A);
}

void trajectory_only_a_abs(struct trajectory *traj, double a_deg_abs)
{
    double posa = position_get_a_rad_double(traj->position);
    double a;

    a = RAD(a_deg_abs) - posa;
    a = modulo_2pi(a);
    __trajectory_goto_d_a_rel(traj, 0, a, RUNNING_A, UPDATE_A);
}

void trajectory_d_a_rel(struct trajectory *traj, double d_mm, double a_deg)
{
    __trajectory_goto_d_a_rel(traj, d_mm, RAD(a_deg),
                              RUNNING_AD, UPDATE_A | UPDATE_D);
}

void trajectory_stop(struct trajectory *traj)
{
    DEBUG(E_TRAJECTORY, "stop");
    __trajectory_goto_d_a_rel(traj, 0, 0, READY,
                              UPDATE_A | UPDATE_D | RESET_D | RESET_A);
}

void trajectory_hardstop(struct trajectory *traj)
{
    struct quadramp_filter *q_d, *q_a;

    DEBUG(E_TRAJECTORY, "hardstop");

    q_d = traj->csm_distance->consign_filter_params;
    q_a = traj->csm_angle->consign_filter_params;
    __trajectory_goto_d_a_rel(traj, 0, 0, READY,
                              UPDATE_A | UPDATE_D | RESET_D | RESET_A);

    q_d->previous_var = 0;
    q_d->previous_out = rs_get_distance(traj->robot);
    q_a->previous_var = 0;
    q_a->previous_out = rs_get_angle(traj->robot);
}

void trajectory_goto_xy_abs(struct trajectory *traj, double x, double y)
{
    DEBUG(E_TRAJECTORY, "Goto XY");
    delete_event(traj);
    traj->target.cart.x = x;
    traj->target.cart.y = y;
    traj->state = RUNNING_XY_START;
    schedule_event(traj);
}

void trajectory_goto_forward_xy_abs(struct trajectory *traj, double x, double y)
{
    DEBUG(E_TRAJECTORY, "Goto XY_F");
    delete_event(traj);
    traj->target.cart.x = x;
    traj->target.cart.y = y;
    traj->state = RUNNING_XY_F_START;
    schedule_event(traj);
}

void trajectory_goto_backward_xy_abs(struct trajectory *traj, double x, double y)
{
    DEBUG(E_TRAJECTORY, "Goto XY_B");
    delete_event(traj);
    traj->target.cart.x = x;
    traj->target.cart.y = y;
    traj->state = RUNNING_XY_B_START;
    schedule_event(traj);
}

void trajectory_goto_d_a_rel(struct trajectory *traj, double d, double a)
{
    vect2_pol p;
    double x = position_get_x_double(traj->position);
    double y = position_get_y_double(traj->position);

    DEBUG(E_TRAJECTORY, "Goto DA rel");

    delete_event(traj);
    p.r = d;
    p.theta = RAD(a) + position_get_a_rad_double(traj->position);
    vect2_pol2cart(&p, &traj->target.cart);
    traj->target.cart.x += x;
    traj->target.cart.y += y;

    traj->state = RUNNING_XY_START;
    schedule_event(traj);
}

void trajectory_goto_xy_rel(struct trajectory *traj, double x_rel_mm, double y_rel_mm)
{
    vect2_cart c;
    vect2_pol p;
    double x = position_get_x_double(traj->position);
    double y = position_get_y_double(traj->position);

    DEBUG(E_TRAJECTORY, "Goto XY rel");

    delete_event(traj);
    c.x = x_rel_mm;
    c.y = y_rel_mm;

    vect2_cart2pol(&c, &p);
    p.theta += position_get_a_rad_double(traj->position);;
    vect2_pol2cart(&p, &traj->target.cart);

    traj->target.cart.x += x;
    traj->target.cart.y += y;

    traj->state = RUNNING_XY_START;
    schedule_event(traj);
}


uint8_t trajectory_angle_finished(struct trajectory *traj)
{
    return cs_get_consign(traj->csm_angle) ==
           cs_get_filtered_consign(traj->csm_angle);
}

uint8_t trajectory_distance_finished(struct trajectory *traj)
{
    if (traj->state == RUNNING_CLITOID_CURVE) {
        return 1;
    }

    return cs_get_consign(traj->csm_distance) ==
           cs_get_filtered_consign(traj->csm_distance);
}

/** return true if the position consign is equal to the filtered
 * position consign (after quadramp filter), for angle and
 * distance. */
uint8_t trajectory_finished(struct trajectory *traj)
{
    return trajectory_distance_finished(traj) && trajectory_angle_finished(traj);

}

/** return true if traj is nearly finished */
uint8_t trajectory_in_window(struct trajectory *traj, double d_win, double a_win_rad)
{
    switch (traj->state) {

        case RUNNING_XY_ANGLE_OK:
        case RUNNING_XY_F_ANGLE_OK:
        case RUNNING_XY_B_ANGLE_OK:
            /* if robot coordinates are near the x,y target */
            return is_robot_in_xy_window(traj, d_win);

        case RUNNING_A:
            return is_robot_in_angle_window(traj, a_win_rad);

        case RUNNING_D:
            return is_robot_in_dist_window(traj, d_win);

        case RUNNING_AD:
            return is_robot_in_dist_window(traj, d_win) &&
                   is_robot_in_angle_window(traj, a_win_rad);

        case RUNNING_XY_START:
        case RUNNING_XY_F_START:
        case RUNNING_XY_B_START:
        case RUNNING_XY_ANGLE:
        case RUNNING_XY_F_ANGLE:
        case RUNNING_XY_B_ANGLE:
        default:
            return 0;
    }
}

/*********** *TRAJECTORY EVENT FUNC */

/** event called for xy trajectories */
void trajectory_manager_xy_event(struct trajectory *traj)
{
    double coef = 1.0;
    double x = position_get_x_double(traj->position);
    double y = position_get_y_double(traj->position);
    double a = position_get_a_rad_double(traj->position);
    int32_t d_consign = 0, a_consign = 0;

    /* These vectors contain target position of the robot in
     * its own coordinates */
    vect2_cart v2cart_pos;
    vect2_pol v2pol_target;

    /* step 1 : process new commands to quadramps */

    switch (traj->state) {
        case RUNNING_XY_START:
        case RUNNING_XY_ANGLE:
        case RUNNING_XY_ANGLE_OK:
        case RUNNING_XY_F_START:
        case RUNNING_XY_F_ANGLE:
        case RUNNING_XY_F_ANGLE_OK:
        case RUNNING_XY_B_START:
        case RUNNING_XY_B_ANGLE:
        case RUNNING_XY_B_ANGLE_OK:

            /* process the command vector from current position to
             * absolute target. */
            v2cart_pos.x = traj->target.cart.x - x;
            v2cart_pos.y = traj->target.cart.y - y;
            vect2_cart2pol(&v2cart_pos, &v2pol_target);
            v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta - a);

            /* asked to go backwards */
            if (traj->state >= RUNNING_XY_B_START &&
                traj->state <= RUNNING_XY_B_ANGLE_OK ) {
                v2pol_target.r = -v2pol_target.r;
                v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta + M_PI);
            }

            /* if we don't need to go forward */
            if (traj->state >= RUNNING_XY_START &&
                traj->state <= RUNNING_XY_ANGLE_OK ) {
                /* If the target is behind the robot, we need to go
                 * backwards. 0.52 instead of 0.5 because we prefer to
                 * go forward */
                if ((v2pol_target.theta > 0.52 * M_PI) ||
                    (v2pol_target.theta < -0.52 * M_PI)) {
                    v2pol_target.r = -v2pol_target.r;
                    v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta + M_PI);
                }
            }

            /* If the robot is correctly oriented to start moving in distance */
            /* here limit dist speed depending on v2pol_target.theta */
            if (fabs(v2pol_target.theta) > traj->a_start_rad) { // || ABS(v2pol_target.r) < traj->d_win)
                set_quadramp_speed(traj, 0, traj->a_speed);
            } else {
                coef = (traj->a_start_rad - fabs(v2pol_target.theta)) / traj->a_start_rad;
                set_quadramp_speed(traj, traj->d_speed * coef, traj->a_speed);
            }


            d_consign = (int32_t)(v2pol_target.r * (traj->position->phys.distance_imp_per_mm));
            d_consign += rs_get_distance(traj->robot);

            /* angle consign */
            /* Here we specify 2.2 instead of 2.0 to avoid oscillations */
            a_consign = (int32_t)(v2pol_target.theta *
                                  (traj->position->phys.distance_imp_per_mm) *
                                  (traj->position->phys.track_mm) / 2.2);
            a_consign += rs_get_angle(traj->robot);

            break;

        default:
            /* hmmm quite odd, delete the event */
            DEBUG(E_TRAJECTORY, "GNI ???");
            delete_event(traj);
            traj->state = READY;
    }


    /* step 2 : update state, or delete event if we reached the
     * destination */
    switch (traj->state) {
        case RUNNING_XY_START:
        case RUNNING_XY_F_START:
        case RUNNING_XY_B_START:
            /* START -> ANGLE */
            DEBUG(E_TRAJECTORY, "-> ANGLE");
            traj->state ++;
            break;

        case RUNNING_XY_ANGLE:
        case RUNNING_XY_F_ANGLE:
        case RUNNING_XY_B_ANGLE: {
            /* if d_speed is not 0, we are in start_angle_win */
            if (get_quadramp_distance_speed(traj)) {
                if (is_robot_in_xy_window(traj, traj->d_win)) {
                    delete_event(traj);
                }
                /* ANGLE -> ANGLE_OK */
                traj->state ++;
                DEBUG(E_TRAJECTORY, "-> ANGLE_OK");
            }
            break;
        }

        case RUNNING_XY_ANGLE_OK:
        case RUNNING_XY_F_ANGLE_OK:
        case RUNNING_XY_B_ANGLE_OK:
            /* If we reached the destination */
            if (is_robot_in_xy_window(traj, traj->d_win)) {
                delete_event(traj);
            }
            break;

        default:
            break;
    }

    /* step 3 : send the processed commands to cs */

    EVT_DEBUG(E_TRAJECTORY,
              "EVENT XY d_cur=%d, d_consign=%d, d_speed=%d, a_cur=%d, a_consign=%d, a_speed=%d",
              rs_get_distance(traj->robot),
              d_consign,
              get_quadramp_distance_speed(traj),
              rs_get_angle(traj->robot),
              a_consign,
              get_quadramp_angle_speed(traj));

    cs_set_consign(traj->csm_angle, a_consign);
    cs_set_consign(traj->csm_distance, d_consign);
}

/*
 * Compute the fastest distance and angle speeds matching the radius
 * from current traj_speed
 */
void circle_get_da_speed_from_radius(struct trajectory *traj,
                                     double radius_mm,
                                     double *speed_d,
                                     double *speed_a)
{
    /* speed_d = coef * speed_a */
    double coef;
    double speed_d2, speed_a2;

    coef = 2. * radius_mm / traj->position->phys.track_mm;

    speed_d2 = traj->a_speed * coef;
    if (speed_d2 < traj->d_speed) {
        *speed_d = speed_d2;
        *speed_a = traj->a_speed;
    } else {
        speed_a2 = traj->d_speed / coef;
        *speed_d = traj->d_speed;
        *speed_a = speed_a2;
    }
}

/* trajectory event for circles */
void trajectory_manager_circle_event(struct trajectory *traj)
{
    double radius;
    double x = position_get_x_double(traj->position);
    double y = position_get_y_double(traj->position);
    double a = position_get_a_rad_double(traj->position);
    int32_t d_consign = 0, a_consign = 0;
    double angle_to_center_rad;
    double coef_p, coef_d;
    double d_speed, a_speed;

    /* These vectors contain target position of the robot in
     * its own coordinates */
    vect2_cart v2cart_pos;
    vect2_pol v2pol_target;

    /* step 1 : process new commands to quadramps */

    /* process the command vector from current position to the
     * center of the circle. */
    v2cart_pos.x = traj->target.circle.center.x - x;
    v2cart_pos.y = traj->target.circle.center.y - y;
    vect2_cart2pol(&v2cart_pos, &v2pol_target);
    v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta - a);

    /* radius consign */
    radius = traj->target.circle.radius;

    coef_p = v2pol_target.r / radius;
    coef_p = 1. * coef_p;

    angle_to_center_rad = v2pol_target.theta - (M_PI / 2.);
    angle_to_center_rad = simple_modulo_2pi(angle_to_center_rad);
    if (angle_to_center_rad > 0.5) {
        angle_to_center_rad = 0.5;
    }
    if (angle_to_center_rad < -0.5) {
        angle_to_center_rad = -0.5;
    }
    coef_d = exp(5 * angle_to_center_rad);

    circle_get_da_speed_from_radius(traj, radius / (coef_p * coef_d),
                                    &d_speed, &a_speed);

    set_quadramp_speed(traj, d_speed, a_speed);

    EVT_DEBUG(E_TRAJECTORY, "angle=%2.2f radius=%2.2f r=%2.2f coef_p=%2.2f coef_d=%2.2f "
                            "d_speed=%2.2f a_speed=%2.2f",
              angle_to_center_rad, radius, v2pol_target.r,
              coef_p, coef_d, d_speed, a_speed);

    a_consign = traj->target.circle.dest_angle;
    d_consign = traj->target.circle.dest_distance;


    /* convert relative angle from imp to rad */
    a = traj->target.circle.dest_angle - rs_get_angle(traj->robot);
    a /= traj->position->phys.distance_imp_per_mm;
    a /= traj->position->phys.track_mm;
    a *= 2.;
    if (fabs(a) < RAD(2)) {
        delete_event(traj);
    }

    cs_set_consign(traj->csm_angle, a_consign);
    cs_set_consign(traj->csm_distance, d_consign);
}

/* trajectory event for lines */
static void trajectory_manager_line_event(struct trajectory *traj)
{
    double x = position_get_x_double(traj->position);
    double y = position_get_y_double(traj->position);
    double a = position_get_a_rad_double(traj->position);
    double advance, dist_to_line;
    point_t robot, proj, target_pt;
    int32_t d_consign = 0, a_consign = 0;
    vect2_cart v2cart_pos;
    vect2_pol v2pol_target;

    robot.x = x;
    robot.y = y;

    /* target point on the line is further on the line */
    proj_pt_line(&robot, &traj->target.line.line, &proj);
    dist_to_line = pt_norm(&robot, &proj);
    if (dist_to_line > traj->target.line.advance) {
        advance = 0;
    } else {
        advance = traj->target.line.advance - dist_to_line;
    }
    target_pt.x = proj.x + advance * cos(traj->target.line.angle);
    target_pt.y = proj.y + advance * sin(traj->target.line.angle);

    /* target vector */
    v2cart_pos.x = target_pt.x - x;
    v2cart_pos.y = target_pt.y - y;
    vect2_cart2pol(&v2cart_pos, &v2pol_target);
    v2pol_target.theta = simple_modulo_2pi(v2pol_target.theta - a);

    /* If the robot is correctly oriented to start moving in distance */
    /* here limit dist speed depending on v2pol_target.theta */
    if (fabs(v2pol_target.theta) > traj->a_start_rad) { // || ABS(v2pol_target.r) < traj->d_win)
        set_quadramp_speed(traj, 0, traj->a_speed);
    } else {
        double coef;
        coef = (traj->a_start_rad - fabs(v2pol_target.theta)) / traj->a_start_rad;
        set_quadramp_speed(traj, traj->d_speed * coef, traj->a_speed);
    }

    /* position consign is infinite */
    d_consign = pos_mm2imp(traj, v2pol_target.r);
    d_consign += rs_get_distance(traj->robot);

    /* angle consign (1.1 to avoid oscillations) */
    a_consign = pos_rd2imp(traj, v2pol_target.theta) / 1.1;
    a_consign += rs_get_angle(traj->robot);

    EVT_DEBUG(E_TRAJECTORY, "target.x=%2.2f target.y=%2.2f, a_consign=%d, d_consign=%d",
              target_pt.x, target_pt.y, a_consign, d_consign);

    cs_set_consign(traj->csm_angle, a_consign);
    cs_set_consign(traj->csm_distance, d_consign);

    /* we reached dest, start clitoid */
    if (traj->state == RUNNING_CLITOID_LINE &&
        xy_norm(proj.x,
                proj.y,
                traj->target.line.turn_pt.x,
                traj->target.line.turn_pt.y) <
        xy_norm(proj.x + cos(traj->target.line.angle),
                proj.y + sin(traj->target.line.angle),
                traj->target.line.turn_pt.x,
                traj->target.line.turn_pt.y)) {
        start_clitoid(traj);
    }
}


/* trajectory event */
void trajectory_manager_thd(void * param)
{
    struct trajectory *traj = (struct trajectory *)param;

    while (1) {
        if (traj->scheduled) {
            switch (traj->state) {
                case RUNNING_XY_START:
                case RUNNING_XY_ANGLE:
                case RUNNING_XY_ANGLE_OK:
                case RUNNING_XY_F_START:
                case RUNNING_XY_F_ANGLE:
                case RUNNING_XY_F_ANGLE_OK:
                case RUNNING_XY_B_START:
                case RUNNING_XY_B_ANGLE:
                case RUNNING_XY_B_ANGLE_OK:
                    trajectory_manager_xy_event(traj);
                    break;

                case RUNNING_CIRCLE:
                    trajectory_manager_circle_event(traj);
                    break;

                case RUNNING_LINE:
                case RUNNING_CLITOID_LINE:
                    trajectory_manager_line_event(traj);
                    break;

                default:
                    break;
            }
        }
#ifndef TESTS
        chThdSleepMilliseconds(TRAJ_EVT_PERIOD);
#endif
    }
}

/*********** *CIRCLE */

/* make the robot orbiting around (x,y) on a circle whose radius is
 * radius_mm, and exit when relative destination angle is reached. The
 * flags set if we go forward or backwards, and CW/CCW. */
void trajectory_circle_rel(struct trajectory *traj,
                           double x, double y,
                           double radius_mm,
                           double rel_a_deg,
                           uint8_t flags)
{
    double dst_angle, dst_distance;

    delete_event(traj);

    traj->target.circle.center.x = x;
    traj->target.circle.center.y = y;
    traj->target.circle.radius = radius_mm;
    traj->target.circle.flags = flags;

    /* convert in steps  */
    dst_angle = RAD(rel_a_deg) *
                (traj->position->phys.distance_imp_per_mm) *
                (traj->position->phys.track_mm) / 2.0;

    dst_distance = RAD(rel_a_deg) * (traj->position->phys.distance_imp_per_mm) * radius_mm;

    traj->target.circle.dest_angle = rs_get_angle(traj->robot);
    traj->target.circle.dest_angle += dst_angle;

    traj->target.circle.dest_distance = rs_get_distance(traj->robot);
    traj->target.circle.dest_distance += dst_distance;

    DEBUG(E_TRAJECTORY, "Circle rel (x,y)=%2.2f,%2.2f r=%2.2f flags=%x dst_angle=%d",
          x, y, radius_mm, flags, traj->target.circle.dest_angle);

    traj->state = RUNNING_CIRCLE;
    schedule_event(traj);
}

/* return the distance in millimeters that corresponds to an angle in
 * degree and a radius in mm */
/* static  */ double circle_get_dist_from_degrees(double radius_mm, double a_deg)
{
    double a_rad = RAD(a_deg);
    return a_rad * radius_mm;
}



/*********** *LINE */

/* Follow a line */
static void __trajectory_line_abs(struct trajectory *traj,
                                  double x1, double y1,
                                  double x2, double y2,
                                  double advance)
{
    point_t p1, p2;

    /* find the line EQ */
    p1.x = x1;
    p1.y = y1;
    p2.x = x2;
    p2.y = y2;
    pts2line(&p1, &p2, &traj->target.line.line);

    /* find the line angle */
    traj->target.line.angle = atan2(y2 - y1, x2 - x1);
    traj->target.line.advance = advance;

    DEBUG(E_TRAJECTORY, "Line rel (a,b,c)=%2.2f,%2.2f,%2.2f",
          traj->target.line.line.a,
          traj->target.line.line.b,
          traj->target.line.line.c,
          traj->target.line.angle);

}

/* Follow a line */
void trajectory_line_abs(struct trajectory *traj,
                         double x1, double y1,
                         double x2, double y2,
                         double advance)
{
    delete_event(traj);
    __trajectory_line_abs(traj, x1, y1, x2, y2, advance);
    traj->state = RUNNING_LINE;
    schedule_event(traj);
}

/*** CLOTHOID */

/**
 * process clitoid parameters
 *
 * - alpha: total angle
 * - beta: circular part of angle (lower than alpha)
 * - R: the radius of the circle (must be != 0)
 * - Vd: linear speed to use (in imp per cs period)
 * - Amax: maximum angular acceleration
 * - d_inter: distance in mm until the intersection of the
 *            2 lines
 *
 * return 0 on success: in this case these parameters are filled:
 * - Aa_out: the angular acceleration to configure in quadramp
 * - Va_out: the angular speed to configure in quadramp
 * - remain_d_mm_out: remaining distance before start to turn
 */
static int8_t calc_clitoid(struct trajectory *traj,
                           double x, double y, double a_rad,
                           double alpha_deg, double beta_deg, double R_mm,
                           double Vd, double Amax, double d_inter_mm,
                           double *Aa_out, double *Va_out, double *remain_d_mm_out)
{
    double Vd_mm_s;
    double Va, Va_rd_s;
    double t, tau, d_mm, alpha_rad, beta_rad;
    double remain_d_mm;
    double Aa, Aa_rd_s2;
    line_t line1, line2;
    line_t line1_int, line2_int;
    point_t robot, intersect, pt2, center, proj, M;
    vect_t v;
    double xm, ym, L, A;

    /* param check */
    if (fabs(alpha_deg) <= fabs(beta_deg)) {
        DEBUG(E_TRAJECTORY, "alpha is smaller than beta");
        return -1;
    }

    /* get angular speed Va */
    Vd_mm_s = speed_imp2mm(traj, Vd);
    DEBUG(E_TRAJECTORY, "Vd_mm_s=%2.2f", Vd_mm_s);
    Va_rd_s = Vd_mm_s / R_mm;
    Va = speed_rd2imp(traj, Va_rd_s);
    DEBUG(E_TRAJECTORY, "Va_rd_s=%2.2f Va=%2.2f", Va_rd_s, Va);

    /* process 't', the time in seconds that we will take to do
     * the first clothoid */
    alpha_rad = RAD(alpha_deg);
    beta_rad = RAD(beta_deg);
    t = fabs(((alpha_rad - beta_rad) * R_mm) / Vd_mm_s);
    DEBUG(E_TRAJECTORY, "R_mm=%2.2f a_rad=%2.2f alpha_rad=%2.2f beta_rad=%2.2f t=%2.2f",
          R_mm, a_rad, alpha_rad, beta_rad, t);

    /* process the angular acceleration */
    Aa_rd_s2 = Va_rd_s / t;
    Aa = acc_rd2imp(traj, Aa_rd_s2);
    DEBUG(E_TRAJECTORY, "Aa_rd_s2=%2.2f Aa=%2.2f", Aa_rd_s2, Aa);

    /* exit if the robot cannot physically do it */
    if (Aa > Amax) {
        DEBUG(E_TRAJECTORY, "greater than max acceleration");
        return -1;
    }

    /* define line1 and line2 */
    robot.x = x;
    robot.y = y;
    intersect.x = x + cos(a_rad) * d_inter_mm;
    intersect.y = y + sin(a_rad) * d_inter_mm;
    pts2line(&robot, &intersect, &line1);
    pt2.x = intersect.x + cos(a_rad + alpha_rad);
    pt2.y = intersect.y + sin(a_rad + alpha_rad);
    pts2line(&intersect, &pt2, &line2);
    DEBUG(E_TRAJECTORY, "intersect=(%2.2f, %2.2f)",
          intersect.x, intersect.y);

    /* L and A are the parameters of the clothoid, xm and ym are
     * the relative coords (starting from the beginning of
     * clothoid) of the crossing point between the clothoid and
     * the circle. */
    L = Vd_mm_s * t;
    A = R_mm * sqrt(fabs(alpha_rad - beta_rad));
    xm =
        L
        - (pow(L, 5) / (40. * pow(A, 4)))
        + (pow(L, 9) / (3456. * pow(A, 8)))
        - (pow(L, 13) / (599040. * pow(A, 12)));
    ym =
        (pow(L, 3) / (6. * pow(A, 2)))
        - (pow(L, 7) / (336. * pow(A, 6)))
        + (pow(L, 11) / (42240. * pow(A, 10)))
        - (pow(L, 15) / (9676800. * pow(A, 14)));
    DEBUG(E_TRAJECTORY, "relative xm,ym = (%2.2f, %2.2f)",
          xm, ym);

    /* the center of the circle is at d_mm when we have to start
     * the clothoid */
    tau = (alpha_rad - beta_rad) / 2.;
    d_mm = ym + (R_mm * cos(tau));
    DEBUG(E_TRAJECTORY, "d_mm=%2.2f", d_mm);

    /* translate line1 */
    memcpy(&line1_int, &line1, sizeof(line1_int));
    memcpy(&line2_int, &line2, sizeof(line2_int));
    v.x = intersect.x - robot.x;
    v.y = intersect.y - robot.y;
    if (alpha_rad > 0) {
        vect_rot_trigo(&v);
    } else {
        vect_rot_retro(&v);
    }
    vect_resize(&v, d_mm);
    line_translate(&line1_int, &v);
    DEBUG(E_TRAJECTORY, "translate line1 by %2.2f,%2.2f", v.x, v.y);

    /* translate line2_int */
    v.x = intersect.x - pt2.x;
    v.y = intersect.y - pt2.y;
    if (alpha_rad < 0) {
        vect_rot_trigo(&v);
    } else {
        vect_rot_retro(&v);
    }
    vect_resize(&v, d_mm);
    line_translate(&line2_int, &v);
    DEBUG(E_TRAJECTORY, "translate line2 by %2.2f,%2.2f", v.x, v.y);

    /* find the center of the circle, at the intersection of the
     * new translated lines */
    if (intersect_line(&line1_int, &line2_int, &center) != 1) {
        DEBUG(E_TRAJECTORY, "cannot find circle center");
        return -1;
    }
    DEBUG(E_TRAJECTORY, "center=(%2.2f,%2.2f)", center.x, center.y);

    /* M is the same point than xm, ym but in absolute coords */
    if (alpha_rad < 0) {
        M.x = center.x + cos(a_rad + M_PI / 2 + tau) * R_mm;
        M.y = center.y + sin(a_rad + M_PI / 2 + tau) * R_mm;
    } else {
        M.x = center.x + cos(a_rad - M_PI / 2 + tau) * R_mm;
        M.y = center.y + sin(a_rad - M_PI / 2 + tau) * R_mm;
    }
    DEBUG(E_TRAJECTORY, "absolute M = (%2.2f, %2.2f)", M.x, M.y);

    /* project M on line 1 */
    proj_pt_line(&M, &line1, &proj);
    DEBUG(E_TRAJECTORY, "proj M = (%2.2f, %2.2f)", proj.x, proj.y);

    /* process remaining distance before start turning */
    remain_d_mm = d_inter_mm - (pt_norm(&proj, &intersect) + xm);
    DEBUG(E_TRAJECTORY, "remain_d=%2.2f", remain_d_mm);
    if (remain_d_mm < 0) {
        DEBUG(E_TRAJECTORY, "too late, cannot turn");
        return -1;
    }

    /* return result */
    *Aa_out = Aa;
    *Va_out = Va;
    *remain_d_mm_out = remain_d_mm;
    return 0;
}

/* after the line, start the clothoid */
static void start_clitoid(struct trajectory *traj)
{
    double Aa = traj->target.line.Aa;
    double Va = traj->target.line.Va;
    double a_rad = traj->target.line.alpha;
    double R_mm = traj->target.line.R;
    double d;

    DEBUG(E_TRAJECTORY, "%s() Va=%2.2f Aa=%2.2f",
          __FUNCTION__, Va, Aa);
    delete_event(traj);
    d = fabs(R_mm * a_rad);
    d *= 3.; /* margin to avoid deceleration */
    trajectory_d_a_rel(traj, d, DEG(a_rad));
    set_quadramp_acc(traj, traj->d_acc, Aa);
    set_quadramp_speed(traj, traj->d_speed, Va);
    traj->state = RUNNING_CLITOID_CURVE;
}


/**
 * do a superb curve joining line1 to line2 which is composed of:
 *   - a clothoid starting from line1
 *   - a circle
 *   - another clothoid up to line2
 * this curve is called a clitoid (hehe)
 *
 * the function assumes that the initial linear speed is Vd and
 * angular speed is 0.
 *
 * - x,y,a_deg: starting position
 * - advance: parameter for line following
 * - alpha: total angle
 * - beta: circular part of angle (lower than alpha)
 * - R: the radius of the circle (must be != 0)
 * - Vd: linear speed to use (in imp per cs period)
 * - Amax: maximum angular acceleration
 * - d_inter: distance in mm until the intersection of the
 *            2 lines
 *
 * return 0 if trajectory can be loaded, then it is processed in
 * background.
 */
int8_t trajectory_clitoid(struct trajectory *traj,
                          double x, double y, double a_deg, double advance,
                          double alpha_deg, double beta_deg, double R_mm,
                          double d_inter_mm)
{
    double remain = 0, Aa = 0, Va = 0, Vd;
    double turnx, turny;
    double a_rad = RAD(a_deg);

    Vd = traj->d_speed;
    if (calc_clitoid(traj, x, y, a_rad, alpha_deg, beta_deg, R_mm,
                     Vd, traj->a_acc, d_inter_mm,
                     &Aa, &Va, &remain) < 0) {
        DEBUG(E_TRAJECTORY, "%s() calc_clitoid returned an error",
              __FUNCTION__);
        return -1;
    }

    delete_event(traj);
    turnx = x + cos(a_rad) * remain;
    turny = y + sin(a_rad) * remain;
    traj->target.line.Aa = Aa;
    traj->target.line.Va = Va;
    traj->target.line.alpha = RAD(alpha_deg);
    traj->target.line.R = R_mm;
    traj->target.line.turn_pt.x = turnx;
    traj->target.line.turn_pt.y = turny;
    DEBUG(E_TRAJECTORY, "%s() turn_pt=%2.2f,%2.2f",
          __FUNCTION__, turnx, turny);

    __trajectory_line_abs(traj, x, y, turnx, turny,
                          advance);
    traj->state = RUNNING_CLITOID_LINE;
    schedule_event(traj);
    return 0;
}
