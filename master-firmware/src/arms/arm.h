#ifndef ARM_API_H
#define ARM_API_H

#include <cstddef>

#include "scara/scara.h"
#include "scara/scara_trajectories.h"

class ArmTrajectory
{
public:
    ArmTrajectory(scara_t* arm);

    size_t size() const;
    scara_waypoint_t frame(int index) const;

    ArmTrajectory& goThrough(float x, float y, float z,
                             scara_coordinate_t coordinate);

private:
    scara_t* m_arm;
    scara_trajectory_t m_trajectory;
};

#endif
