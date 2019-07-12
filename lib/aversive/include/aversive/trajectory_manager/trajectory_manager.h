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
 *  Revision : $Id: trajectory_manager.h,v 1.4.4.10 2009-05-02 10:03:04 zer0 Exp $
 *
 */

/** @file trajectory_manager.h
 *
 * @brief Interface to the trajectory manager.
 *
 * This module handles the high level trajectory management functions, like
 * going from one point to another in a smooth curve, etc...
 *
 * This module only works with a control system made with control_system_manager,
 * position_manager, quadramp, robot_system. It uses the vect2 module.
 */

#ifndef TRAJECTORY_MANAGER
#define TRAJECTORY_MANAGER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <aversive/math/vect2/vect2.h>
#include <aversive/math/geometry/vect_base.h>
#include <aversive/math/geometry/lines.h>
#include <aversive/robot_system/robot_system.h>

/** State of the trajectory manager. */
enum trajectory_state {
    READY, /**< Ready, waiting for a trajectory. */

    /* simple trajectories */
    RUNNING_A, /**< Turning without moving. */
    RUNNING_D, /**< Going straight. */
    RUNNING_AD, /**< Going forward and turning at the same time. */

    /* trajectories using events */
    RUNNING_XY_START, /**< A trajectory order was given, preparing to go. */
    RUNNING_XY_ANGLE, /**< Doing a preliminary turn before starting to move. */
    RUNNING_XY_ANGLE_OK, /**< Angle is now ok, move forward. */
    RUNNING_XY_F_START, /**< Same as RUNNING_XY_START but forward only. */
    RUNNING_XY_F_ANGLE, /**< Same as RUNNING_XY_ANGLE but forward only. */
    RUNNING_XY_F_ANGLE_OK, /**< Same as RUNNING_XY_ANGLE_OK but forward only. */
    RUNNING_XY_B_START, /**< Same as RUNNING_XY_START but backward only. */
    RUNNING_XY_B_ANGLE, /**< Same as RUNNING_XY_ANGLE but backward only. */
    RUNNING_XY_B_ANGLE_OK, /**< Same as RUNNING_XY_B_ANGLE_OK but backward only. */

    /* circle */
    RUNNING_CIRCLE, /**< Running a circle trajectory. */

    /* line */
    RUNNING_LINE, /**< Following a line. */

    /* clitoid */
    RUNNING_CLITOID_LINE, /**< Running a clitoid (line->circle->line) in the line part. */
    RUNNING_CLITOID_CURVE, /**< Running a clitoid in the curve part. */
};

/** Movement target when running on a circle. */
struct circle_target {
    vect2_cart center; /**< center of the circle */
    double radius; /**< radius of the circle */
    int32_t dest_angle; /**< dst angle in inc */
    int32_t dest_distance; /**< dst angle in inc */

#define TRIGO 1 /**< Rotation is counterclockwise */
#define FORWARD 2 /**< go forward or backward */
    uint8_t flags; /**< flags for this trajectory */
};

/** Movement target when running a line or a clitoid. */
struct line_target {
    line_t line; /**< The line to follow. */
    double angle; /**< The angle of line. */
    double advance; /**< The sampling factor. */

    /* only for clitoid */
    point_t turn_pt; /**< The starting point of the clitoid. */
    double Aa; /**< The angular acceleration. */
    double Va; /**< The angular speed. */
    double alpha; /**< The angle to turn. */
    double R; /**< The radius of the circular part. */
};

/** A complete instance of the trajectory manager. */
struct trajectory {
    bool scheduled; // Is any trajectory related action is scheduled

    enum trajectory_state state; /**< describe the type of target, and if we reached the target */

    union {
        vect2_cart cart; /**< target, if it is a x,y vector */
        struct rs_polar pol; /**< target, if it is a d,a vector */
        struct circle_target circle; /**< target, if it is a circle */
        struct line_target line; /**< target, if it is a line */
    } target; /**< Target of the movement. */

    double d_win; /**<< distance window (for END_NEAR) */
    double a_win_rad; /**<< angle window (for END_NEAR) */
    double a_start_rad; /**<< in xy consigns, start to move in distance
                        *   when a_target < a_start */

    double d_speed; /**<< distance speed consign */
    double a_speed; /**<< angle speed consign */

    double d_acc; /**<< distance acceleration consign */
    double a_acc; /**<< angle acceleration consign */

    struct robot_position* position; /**<< associated robot_position */
    struct robot_system* robot; /**<< associated robot_system */
    struct cs* csm_angle; /**<< associated control system (angle) */
    struct cs* csm_distance; /**<< associated control system (distance) */

    double cs_hz; /**< The frequency of the control system associated with this manager. */
};

/** @brief Structure initialization.
 *
 * @param [in] traj The trajectory manager to initialize.
 * @param [in] cs_hz The frequency of the control systems, in Hz. */
void trajectory_manager_init(struct trajectory* traj, double cs_hz);

/** @brief Sets the control systems.
 *
 * This function tells the trajectory manager which control system to use for
 * angle and distance control.
 *
 * @param [in] traj Tje trajectory manager instance.
 * @param [in] cs_d, cs_a The control systems to use.
 */
void trajectory_set_cs(struct trajectory* traj, struct cs* cs_d, struct cs* cs_a);

/** @brief Sets related robot params.
 *
 * Sets the robot_pos and robot_system used for the computation of the trajectory.
 * @param [in] traj The trajectory manager instance.
 * @param [in] rs The related robot system.
 * @param [in] pos The position manager instance of the robot.
 */
void trajectory_set_robot_params(struct trajectory* traj,
                                 struct robot_system* rs,
                                 struct robot_position* pos);

/** @brief Set speed consign.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_speed The distance speed in imp / period.
 * @param [in] a_speed The angular speed in imp / period.
 *
 * @sa speed_mm2imp
 * @sa speed_rd2imp
 */
void trajectory_set_speed(struct trajectory* traj, double d_speed, double a_speed);

/** @brief Set acceleration consign.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_acc The distance acceleration in imp / period^2.
 * @param [in] a_acc The angular acceleration in imp / period^2.
 *
 * @sa acc_mm2imp
 * @sa acc_rd2imp
 */
void trajectory_set_acc(struct trajectory* traj, double d_acc, double a_acc);

/** @brief Set windows for trajectory.
 *
 * This function sets the windows for the trajectory. The distance window
 * is the distance for which we say we reached target point.
 * The angular window is the angle for which we say we have the correct heading
 * (used only when doing angular trajectories).
 * Finally the angular start windows is used at the beginning of an XY movement
 * to decide if we need to turn first before starting the curve.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_win The distance window in mm.
 * @param [in] a_win_deg The angular window in degrees.
 * @param [in] a_start_deg The angular start window in degrees.
 */
void trajectory_set_windows(struct trajectory* traj, double d_win, double a_win_deg, double a_start_deg);

/** @brief Get trajectory manager state.
 *
 * @param [in] traj The trajectory manager instance.
 * @return The state of the trajectory manager.
 */
enum trajectory_state trajectory_get_state(struct trajectory* traj);

/** @brief Returns true if the trajectory is finished.
 *
 * @param [in] traj The trajectory manager instance.
 * @return 1 if the current trajectory is finished, 0 otherwise.
 */
uint8_t trajectory_finished(struct trajectory* traj);

/** @brief Returns true if the trajectory has reached its final angle.
 *
 * @param [in] traj The trajectory manager instance.
 * @return 1 if the current trajectory has reached its angle, 0 otherwise.
 */
uint8_t trajectory_angle_finished(struct trajectory* traj);

/** @brief Returns true if the trajectory has reached its final distance.
 *
 * @param [in] traj The trajectory manager instance.
 * @return 1 if the current trajectory has reached its distance, 0 otherwise.
 */
uint8_t trajectory_distance_finished(struct trajectory* traj);

/** @brief Returns true if the trajectory is in the given window.
 *
 * This function checks the current state of the trajectory manager against the
 * given tolerances and returns true if it is in the tolerance.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_win The distance tolerance in mm.
 * @param [in] a_win_rad The angle window in rad.
 * @return 1 if the trajectory manager is in the given windows.
 */
uint8_t trajectory_in_window(struct trajectory* traj, double d_win, double a_win_rad);

/* simple commands */

/** @brief Do a soft stop.
 *
 * This function tells the trajectory manager to do a soft stop (slow down).
 *
 * @param [in] traj The trajectory manager instance.
 *
 * @note This function takes some time to stop the robot completely. For fast
 * stopping use trajectory_hardstop() instead.
 */
void trajectory_stop(struct trajectory* traj);

/** @brief Do an emergency stop
 *
 * This function tells the trajectory manager to do an emergency stop. To
 * do it, the consign is set as the current position of the robot and the
 * ramps are deactivated.
 *
 * @param [in] traj The trajectory manager instance.
 */
void trajectory_hardstop(struct trajectory* traj);

/** @brief Goes straight.
 *
 * This function makes the robot go straight (angle does not change) for
 * a certain distance.
 *
 * The difference with trajectory_only_d_rel() is that this function forces the
 * robot to stop turning if it is currently on a curve, while
 * trajectory_only_d_rel() doesn't change the angle consign.
 *
 * @sa trajectory_only_d_rel
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_mm The distance, in mm.
 */
void trajectory_d_rel(struct trajectory* traj, double d_mm);

/** @brief Goes straight.
 *
 * This function makes the robot go forward by a certain distance without
 * touching the angle consign.
 *
 * @sa trajectory_d_rel
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_mm The distance consign in mm.
 */
void trajectory_only_d_rel(struct trajectory* traj, double d_mm);

/** @brief Turn by a given angle.
 *
 * This function makes the robot turn by a certain angle (relative turn).
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] a_deg The angle in degrees.
 * @sa trajectory_a_abs
 * @sa trajectory_only_a_abs
 * @sa trajectory_only_a_rel
 */
void trajectory_a_rel(struct trajectory* traj, double a_deg);

/** @brief Turn to a given angle.
 *
 * This function makes the robot turn to a certain angle (absolute turn or heading).
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] a_deg The angle in degrees.
 * @sa trajectory_a_rel
 */
void trajectory_a_abs(struct trajectory* traj, double a_deg);

/** @brief Makes the robot face a given point.
 *
 * This function turns the robot until the given point is in front of it.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_abs_mm, y_abs_mm The coordinate of the point in mm.
 * @sa trajectory_turnto_xy_behind
 */
void trajectory_turnto_xy(struct trajectory* traj, double x_abs_mm, double y_abs_mm);

/** @brief Makes the robot turn its back to a given point.
 *
 * This function turns the robot until the given point is behind it.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_abs_mm, y_abs_mm The coordinate of the point in mm.
 * @sa trajectory_turnto_xy_behind
 */
void trajectory_turnto_xy_behind(struct trajectory* traj, double x_abs_mm, double y_abs_mm);

/** @brief Makes the robot turn by given angle.
 *
 * This function makes the robot turn to a certain angle without changing
 * the distance consign. The difference with trajectory_a_rel() is that the
 * robot is not asked to stop before changing its heading.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] a_deg The angle in degrees.
 * @sa trajectory_a_abs
 * @sa trajectory_a_rel
 * @sa trajectory_only_a_abs
 */
void trajectory_only_a_rel(struct trajectory* traj, double a_deg);

/** @brief Makes the robot turn to given angle.
 *
 * This function makes the robot turn to a certain angle without changing
 * the distance consign. The difference with trajectory_a_abs() is that the
 * robot is not asked to stop before changing its heading.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] a_deg The angle in degrees.
 * @sa trajectory_a_abs
 * @sa trajectory_a_rel
 * @sa trajectory_only_a_rel
 */
void trajectory_only_a_abs(struct trajectory* traj, double a_deg);

/** @brief Makes the robot move and turn.
 *
 * This function makes the robot turn by a_deg and move by d_mm mm at the
 * same time. This can be used to make complex curves.
 *
 * @warning The trajectory made by this function depends on the speed of
 * the robot !
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d_mm The distance to go, in mm.
 * @param [in] a_deg The angle to turn, in degrees.
 */
void trajectory_d_a_rel(struct trajectory* traj, double d_mm, double a_deg);

/* commands using events */

/** @brief Go to a point.
 *
 * This function makes the robot go to a point. Once the function is called, the
 * trajectory manager schedules its own event in the scheduler to regulate on
 * the point. This event is automatically deleted once the point is reached.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_abs_mm, y_abs_mm The coordinates of the point in mm.
 *
 * @note This function allows the robot to go both forward or backward.
 * @sa trajectory_goto_forward_xy_abs
 * @sa trajectory_goto_backward_xy_abs
 */
void trajectory_goto_xy_abs(struct trajectory* traj, double x_abs_mm, double y_abs_mm);

/** @brief Go to a point
 *
 * This function is the same as trajectory_goto_xy_abs() but it forces the robot
 * to go forward.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_abs_mm, y_abs_mm The coordinate of the point.
 * @sa trajectory_goto_xy_abs
 */
void trajectory_goto_forward_xy_abs(struct trajectory* traj, double x_abs_mm, double y_abs_mm);

/** @brief Go to a point
 *
 * This function is the same as trajectory_goto_xy_abs() but it forces the robot
 * to go backward.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_abs_mm, y_abs_mm The coordinate of the point.
 * @sa trajectory_goto_xy_abs
 */
void trajectory_goto_backward_xy_abs(struct trajectory* traj, double x_abs_mm, double y_abs_mm);

/** @brief Go to a point given in polar coordinates.
 *
 * This function goes to the point given by its relative polar coordinates.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] d The distance in mm.
 * @param [in] a The angle in degrees.
 */
void trajectory_goto_d_a_rel(struct trajectory* traj, double d, double a);

/** @brief Go to a relative point.
 *
 * This function goes to the point whose coordinates are given relative to the
 * robot. This means the X axis points toward the front of the robot, while the
 * Y axis points to its left.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x_rel_mm, y_rel_mm The coordinate of the point, in mm.
 */
void trajectory_goto_xy_rel(struct trajectory* traj, double x_rel_mm, double y_rel_mm);

/** make the robot orbiting around (x,y) on a circle whose radius is
 * radius_mm, and exit when relative destination angle is reached. The
 * flags set if we go forward or backwards, and CW/CCW. */

/** @brief Makes the robot orbit around a given point.
 *
 * This function makes the robot orbit around (x, y) on a circle of radius
 * radius_mm and exit when the relative destination angle is reached. The
 * flag passed tells us if we should go forward or backward and clockwise
 * or counterclockwise.
 *
 * @todo Finish and test this function.
 * @warning Do not use this function yet, it was NOT tested at CVRA.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x,y The coordinate of the center of the circle.
 * @param [in] radius_mm The radius of the circle in mm.
 * @param [in] rel_a_deg The relative target angle.
 * @param [in] flags Set of OR'd flags to tell if we should go forward/backward and CW/CCW.
 *
 * @sa FORWARD
 * @sa TRIGO
 */
void trajectory_circle_rel(struct trajectory* traj, double x, double y, double radius_mm, double rel_a_deg, uint8_t flags);

/*
 * Compute the fastest distance and angle speeds matching the radius
 * from current traj_speed
 */
/** @brief Computes the best speed doable to stay on the circle.
 *
 * This function computes the fastest distance and angle speeds matching the given radius
 * using the speeds given in the trajectory manager instance.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] radius_mm The radius of the circle.
 * @param [out] speed_d, speed_a The variable to stock the results.
 */
void circle_get_speed_from_radius(struct trajectory* traj,
                                  double radius_mm,
                                  double* speed_d,
                                  double* speed_a);

/** @brief Do a line.
 *
 * This function makes the robot follow a line indefinitely. If the robot is
 * not on the line when the function is called, the robot will go to the
 * line before following it. The orientation of the line is given by the two
 * points : The line goes from (x1, y1) to (x2, y2).
 *
 * The sampling parameter (advance) is used to compute how much space is between
 * two interpolation points of the line. This coefficient should be hand guessed.
 *
 * @param [in] traj The trajectory manager instance.
 * @param [in] x1, y1 Coordinate of the first point.
 * @param [in] x2, y2 Coordinate of the second point.
 * @param [in] advance The sampling coefficient.
 *
 * @todo Test this function.
 */
void trajectory_line_abs(struct trajectory* traj, double x1, double y1, double x2, double y2, double advance);

#ifdef __cplusplus
}
#endif
#endif // TRAJECTORY_MANAGER
