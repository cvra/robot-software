#include <cmath>
#include <numeric>
#include "Eigen/Dense"

#include "scara_jacobian.h"

Eigen::Matrix2d jacobian_matrix(float alpha, float beta, float l1, float l2)
{
    Eigen::Matrix2d jacobian;
    jacobian <<
        - l1 * sin(alpha) - l2 * sin(alpha + beta),
        - l2 * sin(alpha + beta),
        l1 * cos(alpha) + l2 * cos(alpha + beta),
        l2 * cos(alpha + beta);

    return jacobian;
}

void scara_jacobian_compute(float f_x, float f_y, float alpha, float beta,
                            float l1, float l2, float *torque_alpha,
                            float *torque_beta)
{
    Eigen::Matrix2d J = jacobian_matrix(alpha, beta, l1, l2);
    Eigen::Vector2d force(f_x, f_y);
    const float damping = 2.;

    /* Compute the torque using damped least square */
    Eigen::Vector2d torque =
        J.transpose() *
        (J * J.transpose() + (damping * damping) * Eigen::Matrix2d::Identity())
            .inverse() *
        force;

    *torque_alpha = torque[0];
    *torque_beta = torque[1];
}
