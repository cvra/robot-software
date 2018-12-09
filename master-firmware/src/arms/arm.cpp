#include "arm.h"

ArmTrajectory::ArmTrajectory(scara_t* arm)
    : m_arm(arm)
    , m_duration(0.)
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

float ArmTrajectory::duration() const
{
    return m_duration;
}

ArmTrajectory& ArmTrajectory::startAt(const ArmTrajectoryFrame& frame)
{
    scara_trajectory_init(&m_trajectory);
    scara_trajectory_append_point(&m_trajectory, frame.position,
                                  frame.coordinate,
                                  {.x = 500.f, .y = 500.f, .z = 1000.f},
                                  m_arm->length);
    m_duration = 0.f;
    return *this;
}

ArmTrajectory& ArmTrajectory::goThrough(const ArmTrajectoryFrame& frame)
{
    scara_trajectory_append_point(&m_trajectory, frame.position,
                                  frame.coordinate,
                                  {.x = 500.f, .y = 500.f, .z = 1000.f},
                                  m_arm->length);
    m_duration += 1.f;
    return *this;
}

ArmTrajectoryFrame ArmTrajectory::execute() const
{
    if (size() > 0) {
        return frame(size() - 1);
    } else {
        return ArmTrajectoryFrame();
    }
}
