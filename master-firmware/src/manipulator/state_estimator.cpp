#include <math.h>

#include "manipulator/state_estimator.h"

namespace manipulator {
StateEstimator::StateEstimator(const std::array<float, 3>& link_lengths)
    : lengths(link_lengths)
{
}

Pose2D StateEstimator::get_state() const
{
    return forward_kinematics(lengths, angles);
}

void StateEstimator::update_state(const Angles& new_angles)
{
    angles = new_angles;
}
} // namespace manipulator
