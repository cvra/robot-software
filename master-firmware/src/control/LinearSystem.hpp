#pragma once

#include "Eigen/Dense"

template <int N, int M>
struct LinearSystem {
    Eigen::Matrix<float, N, N> A;
    Eigen::Matrix<float, N, M> B;
};

template <int N, int M>
LinearSystem<N, M> discretize(const LinearSystem<N, M>& system, float sampling_period)
{
    LinearSystem<N, M> discrete;

    Eigen::Matrix<float, N, N> I = Eigen::Matrix<float, N, N>::Identity();

    Eigen::Matrix<float, N, N> tmp = I - 0.5 * sampling_period * system.A;
    discrete.A = (I + 0.5 * sampling_period * system.A) * tmp.inverse();

    discrete.B = sampling_period * system.B;

    return discrete;
}
