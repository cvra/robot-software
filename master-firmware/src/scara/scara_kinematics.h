#ifndef SCARA_KINEMATICS_H
#define SCARA_KINEMATICS_H

#include <stdbool.h>
#include <aversive/math/geometry/vect_base.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "scara_waypoint.h"

typedef enum {
    SHOULDER_FRONT,
    SHOULDER_BACK,
} shoulder_mode_t;

/** Computes the possible positions for the elbow.
 *
 * @param [in] x,y The coordinates of the hand.
 * @param [in] l1,l2 The length of the two parts of the arm.
 * @param [out] p1, p2 The possible positions.
 * @returns The number of possible positons (0, 1 or 2).
 */
int scara_num_possible_elbow_positions(point_t target, float l1, float l2, point_t* p1, point_t* p2);

/** Inverts the shoulder mode depending on wheter the arm is facing the
 * left or right side of the robot. This is needed as "forward" and "backward"
 * are in robot frame while the transformations should be applied in arm frame.
 */
shoulder_mode_t scara_orientation_mode(shoulder_mode_t mode, float scara_angle_offset);

/** Choose the correct elbow solution to avoid colision with the robot and to
 * respect the given mode.
 * @note The mode is for angle_offset > 0. Use scara_orientation_mode to change it
 * if needed.
 */
point_t scara_shoulder_solve(point_t target, point_t elbow1, point_t elbow2, shoulder_mode_t mode);

point_t scara_forward_kinematics(float alpha, float beta, float length[2]);

float scara_compute_shoulder_angle(point_t elbow, point_t hand);
float scara_compute_elbow_angle(point_t elbow, point_t hand);

bool scara_compute_joint_angles(position_3d_t position, shoulder_mode_t mode, const float* length, float* alpha, float* beta);

#ifdef __cplusplus
}
#endif

#endif /* SCARA_KINEMATICS_H */
