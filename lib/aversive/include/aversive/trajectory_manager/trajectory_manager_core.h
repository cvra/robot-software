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

#ifndef TRAJECTORY_MANAGER_CORE_H
#define TRAJECTORY_MANAGER_CORE_H

/**
 * update angle and/or distance
 * this function is not called directly by the user
 *   traj  : pointer to the trajectory structure
 *   d_mm  : distance in mm
 *   a_rad : angle in radian
 *   flags : what to update (UPDATE_A, UPDATE_D)
 */
void __trajectory_goto_d_a_rel(struct trajectory* traj, double d_mm, double a_rad, enum trajectory_state state, uint8_t flags) EXCLUSIVE_LOCKS_REQUIRED(traj->lock_) SHARED_LOCKS_REQUIRED(traj->position->lock_);

/** go straight forward (d is in mm) */
void trajectory_d_rel(struct trajectory* traj, double d_mm) LOCKS_EXCLUDED(traj->lock_);

/** update distance consign without changing angle consign */
void trajectory_only_d_rel(struct trajectory* traj, double d_mm) LOCKS_EXCLUDED(traj->lock_);

/** turn by 'a' degrees */
void trajectory_a_rel(struct trajectory* traj, double a_deg_rel) LOCKS_EXCLUDED(traj->lock_);

/** turn by 'a' degrees */
void trajectory_a_abs(struct trajectory* traj, double a_deg_abs) LOCKS_EXCLUDED(traj->lock_);

/** turn the robot until the point x,y is in front of us */
void trajectory_turnto_xy(struct trajectory* traj, double x_abs_mm, double y_abs_mm) LOCKS_EXCLUDED(traj->lock_);

/** turn the robot until the point x,y is behind us */
void trajectory_turnto_xy_behind(struct trajectory* traj, double x_abs_mm, double y_abs_mm) LOCKS_EXCLUDED(traj->lock_);

/** update angle consign without changing distance consign */
void trajectory_only_a_rel(struct trajectory* traj, double a_deg) LOCKS_EXCLUDED(traj->lock_);

/** update angle consign without changing distance consign */
void trajectory_only_a_abs(struct trajectory* traj, double a_deg_abs) LOCKS_EXCLUDED(traj->lock_);

/** turn by 'a' degrees */
void trajectory_d_a_rel(struct trajectory* traj, double d_mm, double a_deg) LOCKS_EXCLUDED(traj->lock_);

/** set relative angle and distance consign to 0 */
void trajectory_stop(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);

/** set relative angle and distance consign to 0, and break any
 * deceleration ramp in quadramp filter */
void trajectory_hardstop(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);

/************ GOTO XY, USE EVENTS */

/** goto a x,y point, using a trajectory event */
void trajectory_goto_xy_abs(struct trajectory* traj, double x, double y) LOCKS_EXCLUDED(traj->lock_);

/** go forward to a x,y point, using a trajectory event */
void trajectory_goto_forward_xy_abs(struct trajectory* traj, double x, double y) LOCKS_EXCLUDED(traj->lock_);

/** go backward to a x,y point, using a trajectory event */
void trajectory_goto_backward_xy_abs(struct trajectory* traj, double x, double y) LOCKS_EXCLUDED(traj->lock_);

/** go forward to a d,a point, using a trajectory event */
void trajectory_goto_d_a_rel(struct trajectory* traj, double d, double a) LOCKS_EXCLUDED(traj->lock_);

/** go forward to a x,y relative point, using a trajectory event */
void trajectory_goto_xy_rel(struct trajectory* traj, double x_rel_mm, double y_rel_mm) LOCKS_EXCLUDED(traj->lock_);

/************ FUNCS FOR GETTING TRAJ STATE */

uint8_t trajectory_angle_finished(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);
uint8_t trajectory_distance_finished(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);

/** return true if the position consign is equal to the filtered
 * position consign (after quadramp filter), for angle and
 * distance. */
uint8_t trajectory_finished(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);
uint8_t trajectory_nearly_finished(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);

/** return true if traj is nearly finished */
uint8_t trajectory_in_window(struct trajectory* traj, double d_win, double a_win_rad) LOCKS_EXCLUDED(traj->lock_);

/*********** *TRAJECTORY EVENT FUNC */

/** event called for xy trajectories */

void trajectory_manager_xy_event(struct trajectory* traj) EXCLUSIVE_LOCKS_REQUIRED(traj->lock_) SHARED_LOCKS_REQUIRED(traj->position->lock_);

/* trajectory event for circles */
void trajectory_manager_circle_event(struct trajectory* traj) EXCLUSIVE_LOCKS_REQUIRED(traj->lock_) SHARED_LOCKS_REQUIRED(traj->position->lock_);

/* trajectory manage events */
void trajectory_manager_manage(struct trajectory* traj) LOCKS_EXCLUDED(traj->lock_);

/*********** *CIRCLE */

/*********** CLITOID */

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
 * - x,y,a: starting position
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
int8_t trajectory_clitoid(struct trajectory* traj,
                          double x,
                          double y,
                          double a,
                          double advance,
                          double alpha_deg,
                          double beta_deg,
                          double R_mm,
                          double d_inter_mm) LOCKS_EXCLUDED(traj->lock_);

#endif /* TRAJECTORY_MANAGER_CORE_H */
