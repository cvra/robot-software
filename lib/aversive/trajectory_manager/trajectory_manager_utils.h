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

#ifndef TRAJECTORY_MANAGER_UTILS_H
#define TRAJECTORY_MANAGER_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <aversive/trajectory_manager/trajectory_manager.h>

#define M_2PI (2 * M_PI)

#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * (M_PI / 180.0))

/** set speed consign in quadramp filter */
void set_quadramp_speed(struct trajectory* traj, double d_speed, double a_speed);

/** set acc consign in quadramp filter */
void set_quadramp_acc(struct trajectory* traj, double d_acc, double a_acc);

/** get angle speed consign in quadramp filter */
double get_quadramp_angle_speed(struct trajectory* traj);

/** get distance speed consign in quadramp filter */
double get_quadramp_distance_speed(struct trajectory* traj);

/** remove event if any */
void delete_event(struct trajectory* traj);

/** schedule the trajectory event */
void schedule_event(struct trajectory* traj);

/** do a modulo 2.pi -> [-Pi,+Pi], knowing that 'a' is in [-3Pi,+3Pi] */
double simple_modulo_2pi(double a);

/** do a modulo 2.pi -> [-Pi,+Pi] */
double modulo_2pi(double a);

/** near the target (dist) ? */
uint8_t is_robot_in_dist_window(struct trajectory* traj, double d_win);

/** near the target (dist in x,y) ? */
uint8_t is_robot_in_xy_window(struct trajectory* traj, double d_win);

/** near the angle target in radian ? Only valid if
 *  traj->target.pol.angle is set (i.e. an angle command, not an xy
 *  command) */
uint8_t is_robot_in_angle_window(struct trajectory* traj, double a_win_rad);

double pos_mm2imp(struct trajectory* traj, double pos);
double pos_imp2mm(struct trajectory* traj, double pos);
double speed_mm2imp(struct trajectory* traj, double speed);
double speed_imp2mm(struct trajectory* traj, double speed);
double acc_mm2imp(struct trajectory* traj, double acc);
double acc_imp2mm(struct trajectory* traj, double acc);
double pos_rd2imp(struct trajectory* traj, double pos);
double pos_imp2rd(struct trajectory* traj, double pos);
double speed_rd2imp(struct trajectory* traj, double speed);
double speed_imp2rd(struct trajectory* traj, double speed);
double acc_rd2imp(struct trajectory* traj, double acc);
double acc_imp2rd(struct trajectory* traj, double acc);
int trajectory_moving_backward(struct trajectory* traj);
int trajectory_moving_forward(struct trajectory* traj);
int trajectory_turning(struct trajectory* traj);

#ifdef __cplusplus
}
#endif

#endif
