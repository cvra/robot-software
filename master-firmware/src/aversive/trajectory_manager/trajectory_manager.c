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
#include <math.h>

#include <math/vect2/vect2.h>

#include <position_manager/position_manager.h>
#include <robot_system/robot_system.h>
#include <control_system_manager/control_system_manager.h>
#include <quadramp/quadramp.h>

#include <trajectory_manager/trajectory_manager.h>
#include "trajectory_manager/trajectory_manager_utils.h"

/************ INIT FUNCS */

/** structure initialization */
void trajectory_manager_init(struct trajectory *traj, double cs_hz)
{
    memset(traj, 0, sizeof(struct trajectory));
    traj->cs_hz = cs_hz;
    traj->state = READY;
}

/** structure initialization */
void trajectory_set_cs(struct trajectory *traj, struct cs *cs_d,
                       struct cs *cs_a)
{
    traj->csm_distance = cs_d;
    traj->csm_angle = cs_a;
}

/** structure initialization */
void trajectory_set_robot_params(struct trajectory *traj,
                                 struct robot_system *rs,
                                 struct robot_position *pos)
{
    traj->robot = rs;
    traj->position = pos;
}

/** set speed consign */
void trajectory_set_speed(struct trajectory *traj, double d_speed, double a_speed)
{
    traj->d_speed = d_speed;
    traj->a_speed = a_speed;
}

/** set acc consign */
void trajectory_set_acc(struct trajectory *traj, double d_acc, double a_acc)
{
    traj->d_acc = d_acc;
    traj->a_acc = a_acc;
}

/** set windows for trajectory */
void trajectory_set_windows(struct trajectory *traj, double d_win,
                            double a_win_deg, double a_start_deg)
{
    traj->d_win = d_win;
    traj->a_win_rad = RAD(a_win_deg);
    traj->a_start_rad = RAD(a_start_deg);
}
