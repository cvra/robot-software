#include "arm.h"

ArmTrajectory::ArmTrajectory(scara_t* arm)
    : m_arm(arm)
{
    scara_trajectory_init(&m_trajectory);
}

size_t ArmTrajectory::size() const
{
    return m_trajectory.frame_count;
}

ArmTrajectoryFrame ArmTrajectory::frame(int index) const
{
    return {m_trajectory.frames[index].position,
            m_trajectory.frames[index].coordinate_type};
}

ArmTrajectory& ArmTrajectory::startAt(const ArmTrajectoryFrame& frame)
{
    scara_trajectory_init(&m_trajectory);
    scara_trajectory_append_point(&m_trajectory, frame.position,
                                  frame.coordinate, 0.0, m_arm->length);
    return *this;
}

ArmTrajectory& ArmTrajectory::goThrough(const ArmTrajectoryFrame& frame)
{
    scara_trajectory_append_point(&m_trajectory, frame.position,
                                  frame.coordinate, 1.0, m_arm->length);
    return *this;
}
