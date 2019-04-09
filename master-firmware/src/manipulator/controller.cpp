#include "manipulator/controller.h"

namespace manipulator {
Controller::Controller(const std::array<float, 3>& link_lengths)
    : lengths(link_lengths)
{
    // for (auto& filter : filters) {

    // }
}

void Controller::set_consign(const Pose2D& target_pose)
{
    target = target_pose;
    target_angles = inverse_kinematics(lengths, target_pose);
}

Angles Controller::compute_input(const Pose2D& state)
{
    measured = state;
    return target_angles;
}

Pose2D Controller::control_error() const
{
    return target - measured;
}
} // namespace manipulator
