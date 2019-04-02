#include <math.h>

#include "manipulator/state_estimator.h"

namespace manipulator {
StateEstimator::StateEstimator(const std::array<float, 3>& link_lengths)
    : lengths(link_lengths)
{
}

Pose2D StateEstimator::get() const
{
    return forward_kinematics(lengths, angles);
}

void StateEstimator::update(const std::array<float, 3>& new_angles)
{
    angles = new_angles;
}
} // namespace manipulator
