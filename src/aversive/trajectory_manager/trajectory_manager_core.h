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

/**
 * update angle and/or distance
 * this function is not called directly by the user
 *   traj  : pointer to the trajectory structure
 *   d_mm  : distance in mm
 *   a_rad : angle in radian
 *   flags : what to update (UPDATE_A, UPDATE_D)
 */
void __trajectory_goto_d_a_rel(struct trajectory *traj, double d_mm,
			       double a_rad, uint8_t state, uint8_t flags);

/** go straight forward (d is in mm) */
void trajectory_d_rel(struct trajectory *traj, double d_mm);

/** update distance consign without changing angle consign */
void trajectory_only_d_rel(struct trajectory *traj, double d_mm);

/** turn by 'a' degrees */
void trajectory_a_rel(struct trajectory *traj, double a_deg_rel);

/** turn by 'a' degrees */
void trajectory_a_abs(struct trajectory *traj, double a_deg_abs);

/** turn the robot until the point x,y is in front of us */
void trajectory_turnto_xy(struct trajectory *traj, double x_abs_mm, double y_abs_mm);


/** turn the robot until the point x,y is behind us */
void trajectory_turnto_xy_behind(struct trajectory *traj, double x_abs_mm, double y_abs_mm);


/** update angle consign without changing distance consign */
void trajectory_only_a_rel(struct trajectory *traj, double a_deg);

/** update angle consign without changing distance consign */
void trajectory_only_a_abs(struct trajectory *traj, double a_deg_abs);


/** turn by 'a' degrees */
void trajectory_d_a_rel(struct trajectory *traj, double d_mm, double a_deg);

/** set relative angle and distance consign to 0 */
void trajectory_stop(struct trajectory *traj);

/** set relative angle and distance consign to 0, and break any
 * deceleration ramp in quadramp filter */
void trajectory_hardstop(struct trajectory *traj);


/************ GOTO XY, USE EVENTS */

/** goto a x,y point, using a trajectory event */
void trajectory_goto_xy_abs(struct trajectory *traj, double x, double y);

/** go forward to a x,y point, using a trajectory event */
void trajectory_goto_forward_xy_abs(struct trajectory *traj, double x, double y);

/** go backward to a x,y point, using a trajectory event */
void trajectory_goto_backward_xy_abs(struct trajectory *traj, double x, double y);

/** go forward to a d,a point, using a trajectory event */
void trajectory_goto_d_a_rel(struct trajectory *traj, double d, double a);

/** go forward to a x,y relative point, using a trajectory event */
void trajectory_goto_xy_rel(struct trajectory *traj, double x_rel_mm, double y_rel_mm);

/************ FUNCS FOR GETTING TRAJ STATE */

uint8_t trajectory_angle_finished(struct trajectory *traj);
uint8_t trajectory_distance_finished(struct trajectory *traj);

/** return true if the position consign is equal to the filtered
 * position consign (after quadramp filter), for angle and
 * distance. */
uint8_t trajectory_finished(struct trajectory *traj);

/** return true if traj is nearly finished */
uint8_t trajectory_in_window(struct trajectory *traj, double d_win, double a_win_rad);

/*********** *TRAJECTORY EVENT FUNC */

/** event called for xy trajectories */

void trajectory_manager_xy_event(struct trajectory *traj);

/* trajectory event for circles */
void trajectory_manager_circle_event(struct trajectory *traj);

/* trajectory event */
void trajectory_manager_event(void * param);

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
int8_t trajectory_clitoid(struct trajectory *traj,
			  double x, double y, double a, double advance,
			  double alpha_deg, double beta_deg, double R_mm,
			  double d_inter_mm);
