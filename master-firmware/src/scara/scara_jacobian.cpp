#include <cmath>
#include <numeric>
#include "Eigen/Dense"

#include "scara_jacobian.h"

Eigen::Matrix4d jacobian_matrix(float alpha, float beta, float gamma, float delta, float l1, float l2, float l3)
{
    (void)delta;
    Eigen::Matrix4d jacobian;
    jacobian <<
        - l1 * sin(alpha) - l2 * sin(alpha + beta) - l3 * sin(alpha + beta + gamma),
        - l2 * sin(alpha + beta) - l3 * sin(alpha + beta + gamma),
        - l3 * sin(alpha + beta + gamma),
        0,
        l1 * cos(alpha) + l2 * cos(alpha + beta) + l3 * cos(alpha + beta + gamma),
        l2 * cos(alpha + beta) + l3 * cos(alpha + beta + gamma),
        l3 * cos(alpha + beta + gamma),
        0,
        1, 1, 1, 0,
        0, 0, 0, 1;

    return jacobian;
}

void scara_jacobian_compute(float f_x, float f_y, float f_theta, float f_pitch,
                            float alpha, float beta, float gamma, float delta,
                            float l1, float l2, float l3,
                            float* torque_alpha, float* torque_beta, float* torque_gamma, float* torque_delta)
{
    Eigen::Matrix4d J = jacobian_matrix(alpha, beta, gamma, delta, l1, l2, l3);
    Eigen::Vector4d force(f_x, f_y, f_theta, f_pitch);
    const float damping = 2.;

    /* Compute the torque using damped least square */
    Eigen::Vector4d torque = J.transpose() * (J * J.transpose()
            + (damping * damping) * Eigen::Matrix4d::Identity()
            ).inverse() * force;

    *torque_alpha = torque[0];
    *torque_beta = torque[1];
    *torque_gamma = torque[2];
    *torque_delta = torque[3];
}
