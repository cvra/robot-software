#ifndef MANIPULATOR_CONTROLLER_H
#define MANIPULATOR_CONTROLLER_H

#include "manipulator/state_estimator.h"

namespace manipulator {
// Cartesian controller of the arm manipulator
struct Controller {
    Pose2D target, measured;
    std::array<float, 3> target_angles = {{0.f, 0.f, 0.f}};
    std::array<float, 3> lengths = {{0.f, 0.f, 0.f}};

    explicit Controller(const std::array<float, 3>& link_lengths);

    void set(const Pose2D& target_pose);
    std::array<float, 3> update(const Pose2D& state);
    Pose2D error() const;
};
} // namespace manipulator

#endif /* MANIPULATOR_CONTROLLER_H */
