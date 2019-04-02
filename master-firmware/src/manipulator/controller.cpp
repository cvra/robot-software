#include "manipulator/controller.h"

namespace manipulator {
Controller::Controller(const std::array<float, 3>& link_lengths)
    : lengths(link_lengths)
{
}

void Controller::set(const Pose2D& target_pose)
{
    target = target_pose;
    target_angles = inverse_kinematics(lengths, target_pose);
}

std::array<float, 3> Controller::update(const Pose2D& state)
{
    measured = state;
    return target_angles;
}

Pose2D Controller::error() const
{
    return target - measured;
}
} // namespace manipulator
