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
    result.x = cos(alpha) * length[0] + cos(alpha + beta) * length[1];
    result.y = sin(alpha) * length[0] + sin(alpha + beta) * length[1];

    return result;
}
