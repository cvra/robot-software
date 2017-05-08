#include <cmath>
#include <numeric>
#include "Eigen/Dense"
// #include "Eigen/SVD"

#include "scara_jacobian.h"

// #include <iostream>

// template<typename _Matrix_Type_>
// _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
// {
//     Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
//     double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
//     return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
// }

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
    Eigen::Matrix3d jacobian = jacobian_matrix(alpha, beta, gamma, l1, l2, l3);

    // std::cout << "det " << jacobian.determinant() << std::endl;

    if (fabs(jacobian.determinant()) < 100.f) {
        jacobian = jacobian_matrix(alpha + 0.1, beta + 0.1, gamma + 0.1, l1, l2, l3);
    }

    // std::cout << "det " << jacobian.determinant() << std::endl;
    // std::cout << "J11 " << - l1 * sin(alpha) - l2 * sin(alpha + beta) - l3 * sin(alpha + beta + gamma) << std::endl;
    // std::cout << "J\n" << jacobian << std::endl;

    Eigen::Vector3d force(f_x, f_y, f_theta);
    Eigen::Vector3d torque = jacobian.inverse() * force;
    // Eigen::Vector3f torque = pseudoInverse<Eigen::Matrix3f>(jacobian) * force;

    // std::cout << "J^-1\n" << jacobian.inverse() << std::endl;

    *torque_alpha = torque[0];
    *torque_beta = torque[1];
    *torque_gamma = torque[2];
}
