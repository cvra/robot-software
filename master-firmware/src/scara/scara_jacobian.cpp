#include <cmath>
#include <numeric>
#include "Eigen/Dense"

#include "scara_jacobian.h"

Eigen::Matrix3d jacobian_matrix(float alpha, float beta, float gamma, float l1, float l2, float l3)
{
    Eigen::Matrix3d jacobian;
    jacobian <<
        - l1 * sin(alpha) - l2 * sin(alpha + beta) - l3 * sin(alpha + beta + gamma),
        - l2 * sin(alpha + beta) - l3 * sin(alpha + beta + gamma),
        - l3 * sin(alpha + beta + gamma),
        l1 * cos(alpha) + l2 * cos(alpha + beta) + l3 * cos(alpha + beta + gamma),
        l2 * cos(alpha + beta) + l3 * cos(alpha + beta + gamma),
        l3 * cos(alpha + beta + gamma),
        1, 1, 1;

    return jacobian;
}

void scara_jacobian_compute(float f_x, float f_y, float f_theta,
                            float alpha, float beta, float gamma,
                            float l1, float l2, float l3,
                            float* torque_alpha, float* torque_beta, float* torque_gamma)
{
    Eigen::Matrix3d J = jacobian_matrix(alpha, beta, gamma, l1, l2, l3);
    Eigen::Vector3d force(f_x, f_y, f_theta);
    const float damping = 2.;

    /* Compute the torque using damped least square */
    Eigen::Vector3d torque = J.transpose() * (J * J.transpose()
            + (damping * damping) * Eigen::Matrix3d::Identity()
            ).inverse() * force;

    *torque_alpha = torque[0];
    *torque_beta = torque[1];
    *torque_gamma = torque[2];
}
