#ifndef ARM_API_H
#define ARM_API_H

#include <cstddef>

#include "scara/scara.h"
#include "scara/scara_trajectories.h"

struct ArmTrajectoryFrame
{
    position_3d_t position;
    scara_coordinate_t coordinate;
};

class ArmTrajectory
{
public:
    ArmTrajectory(scara_t* arm);

    size_t size() const;
    ArmTrajectoryFrame frame(int index) const;

    ArmTrajectory& startAt(const ArmTrajectoryFrame& frame);
    ArmTrajectory& goThrough(const ArmTrajectoryFrame& frame);
    ArmTrajectoryFrame execute() const;

private:
    scara_t* m_arm;
    scara_trajectory_t m_trajectory;
};

#endif
