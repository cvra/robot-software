#ifndef MANIPULATOR_STATE_ESTIMATOR_H
#define MANIPULATOR_STATE_ESTIMATOR_H

#include "manipulator/kinematics.h"

#include <golem/state_estimator.h>

namespace manipulator {
// Simply solves forward kinematics for the system
struct StateEstimator : public golem::StateEstimator<StateEstimator, Pose2D, Angles> {
    std::array<float, 3> lengths = {{0.f, 0.f, 0.f}};
    Angles angles = {{0.f, 0.f, 0.f}};

    explicit StateEstimator(const std::array<float, 3>& link_lengths);
    Pose2D get_state() const;
    void update_state(const Angles& new_angles);
};
} // namespace manipulator

#endif /* MANIPULATOR_STATE_ESTIMATOR_H */
