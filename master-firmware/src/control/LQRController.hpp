#pragma once

#include "Eigen/Dense"

#include "LinearSystem.hpp"

template <int n, int p, int horizon>
struct LQRController {
    LinearSystem<n, p> system; // discrete system
    Eigen::Matrix<float, n, n> Q; // state design matrix
    Eigen::Matrix<float, p, p> R; // input design matrix
    Eigen::Matrix<float, n, p> N; // mixed design matrix
    float sampling_period;

    // Solve Ricatti equation over the horizon
    std::array<Eigen::Matrix<float, n, n>, horizon + 1> ricatti_P() const
    {
        std::array<Eigen::Matrix<float, n, n>, horizon + 1> P;

        P[0] = Eigen::Matrix<float, n, n>::Zero();
        P[horizon] = Q;

        for (int k = horizon - 1; k > 0; k--) {
            auto tmp = R + system.B.transpose() * P[k + 1] * system.B;

            P[k] = system.A.transpose() * P[k + 1] * system.A
                - (system.A.transpose() * P[k + 1] * system.B + N) * tmp.inverse()
                    * (system.B.transpose() * P[k + 1] * system.A + N.transpose())
                + Q;
        }

        return P;
    }

    // Compute the gain over the horizon
    // Internally solves the Ricatti equation
    std::array<Eigen::Matrix<float, p, n>, horizon> gain() const
    {
        std::array<Eigen::Matrix<float, p, n>, horizon> F;

        auto P = ricatti_P();

        for (int k = 0; k < horizon; k++) {
            auto tmp = R + system.B.transpose() * P[k + 1] * system.B;
            F[k] = tmp.inverse() * (system.B.transpose() * P[k + 1] * system.A + N.transpose());
        }

        return F;
    }
};
