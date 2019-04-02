#ifndef MANIPULATOR_STATE_ESTIMATOR_H
#define MANIPULATOR_STATE_ESTIMATOR_H

#include <array>
#include <math.h>

#include "manipulator/kinematics.h"

namespace manipulator {
// Simply solves forward kinematics for the system
struct StateEstimator {
    std::array<float, 3> lengths{0.f, 0.f, 0.f};
    std::array<float, 3> angles{0.f, 0.f, 0.f};

    explicit StateEstimator(const std::array<float, 3>& link_lengths);
    Pose2D get() const;
    void update(const std::array<float, 3>& new_angles);
};
} // namespace manipulator

#endif /* MANIPULATOR_STATE_ESTIMATOR_H */
