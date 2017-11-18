#include <math.h>
#include <aversive/math/geometry/circles.h>

#include "scara_port.h"
#include "scara_kinematics.h"


int scara_num_possible_elbow_positions(point_t target, float l1, float l2, point_t *p1, point_t *p2)
{
    circle_t c1, c2;

    c1.x = 0;
    c1.y = 0;
    c1.r = l1;

    c2.x = target.x;
    c2.y = target.y;
    c2.r = l2;

    return circle_intersect(&c1, &c2, p1, p2);
}

shoulder_mode_t scara_orientation_mode(shoulder_mode_t mode, float scara_angle_offset)
{
    if (scara_angle_offset > 0.)
        return mode;

    if (mode == SHOULDER_BACK)
        return SHOULDER_FRONT;

    return SHOULDER_BACK;
}

point_t scara_shoulder_solve(point_t target, point_t elbow1, point_t elbow2, shoulder_mode_t mode)
{

    if (target.x < 0) {
        if (elbow1.x > elbow2.x)
            return elbow1;
        else
            return elbow2;
    }

    if (mode == SHOULDER_BACK) {
        if (elbow1.y > elbow2.y)
            return elbow1;
        else
            return elbow2;
    } else {
        if (elbow2.y > elbow1.y)
            return elbow1;
        else
            return elbow2;
    }

}

float scara_compute_shoulder_angle(point_t elbow, point_t hand)
{
    (void)hand;

    return atan2f(elbow.y, elbow.x);
}

float scara_compute_elbow_angle(point_t elbow, point_t hand)
{
    float dx, dy;
    dx = hand.x - elbow.x;
    dy = hand.y - elbow.y;
    return atan2f(dy, dx); // tres tres sensible aux erreurs d'arrondis
}

point_t scara_forward_kinematics(float alpha, float beta, float length[2])
{
    point_t result;
    result.x = cosf(alpha) * length[0] + cosf(alpha + beta) * length[1];
    result.y = sinf(alpha) * length[0] + sinf(alpha + beta) * length[1];

    return result;
}

bool scara_compute_joint_angles(position_3d_t position, shoulder_mode_t mode,
                                float* length, float* alpha, float* beta)
{
    point_t target = {.x = position.x, .y = position.y};

    point_t p1, p2;
    int kinematics_solution_count = scara_num_possible_elbow_positions(target, length[0], length[1], &p1, &p2);

    if (kinematics_solution_count == 0) {
        return false;
    } else if (kinematics_solution_count == 2) {
        p1 = scara_shoulder_solve(target, p1, p2, mode);
    }

    /* p1 now contains the correct elbow pos. */
    *alpha = scara_compute_shoulder_angle(p1, target);
    *beta = scara_compute_elbow_angle(p1, target);

    /* This is due to mecanical construction of the arms. */
    *beta = *beta - *alpha;

    /* The arm cannot make one full turn. */
    if (*beta < -M_PI) {
        *beta = 2 * M_PI + *beta;
    }
    if (*beta > M_PI) {
        *beta = *beta - 2 * M_PI;
    }

    return true;
}
