#include "manipulator/kinematics.h"
#include "manipulator/scara_kinematics.h"

#include <error/error.h>

#include <math.h>

namespace manipulator {
Pose2D operator-(const Pose2D& lhs, const Pose2D& rhs)
{
    Pose2D difference;
    difference.x = lhs.x - rhs.x;
    difference.y = lhs.y - rhs.y;
    difference.heading = lhs.heading - rhs.heading;
    return difference;
}

Pose2D forward_kinematics(const ArmLengths& lengths, const Angles& angles)
{
    Pose2D pose;

    pose.x = lengths[0] * cosf(angles[0])
        + lengths[1] * cosf(angles[0] + angles[1])
        + lengths[2] * cosf(angles[0] + angles[1] + angles[2]);
    pose.y = lengths[0] * sinf(angles[0])
        + lengths[1] * sinf(angles[0] + angles[1])
        + lengths[2] * sinf(angles[0] + angles[1] + angles[2]);
    pose.heading = angles[0] + angles[1] + angles[2];

    return pose;
}

Angles inverse_kinematics(const ArmLengths& lengths, const Pose2D& pose)
{
    float x = pose.x - lengths[2] * cosf(pose.heading);
    float y = pose.y - lengths[2] * sinf(pose.heading);

    Angles angles;

    // now we can solve inverse kinematics in 2D
    shoulder_mode_t mode = y < 0 ? SHOULDER_BACK : SHOULDER_FRONT;
    if (!scara_compute_joint_angles({x, y}, mode, lengths.data(), &angles[0], &angles[1])) {
        WARNING("No inverse kinematics solution found");
    }
    angles[2] = pose.heading - (angles[0] + angles[1]);

    return angles;
}

Angles axes_couple(const Angles& decoupled)
{
    Angles coupled;

    coupled[0] = decoupled[0];
    coupled[1] = decoupled[1] - coupled[0];
    coupled[2] = decoupled[2] - coupled[1] - coupled[0];

    return coupled;
}

Angles axes_decouple(const Angles& coupled)
{
    Angles decoupled;

    decoupled[0] = coupled[0];
    decoupled[1] = coupled[1] + coupled[0];
    decoupled[2] = coupled[2] + coupled[1] + coupled[0];

    return decoupled;
}
} // namespace manipulator
