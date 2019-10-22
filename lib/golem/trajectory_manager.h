#ifndef GOLEM_TRAJECTORY_MANAGER_H
#define GOLEM_TRAJECTORY_MANAGER_H

#include "crtp.h"

namespace golem {
enum class TrajectoryManagerStatus : int {
    Ready = 0, // read to receive a new target
    Moving, // moving towards the set target
};

// Models a TrajectoryManager
// Accepts any class that implements the following functions for given State and Consign types:
// - Consign set_target(const State&);
// - TrajectoryManagerStatus manage_trajectory(const State&);
template <typename TrajectoryManagerImpl, typename State, typename Consign>
class TrajectoryManager : public Crtp<TrajectoryManagerImpl, TrajectoryManager> {
protected:
    State current_target;

public:
    Consign set(const State& target)
    {
        current_target = target;
        return this->underlying().set_target(target);
    }
    TrajectoryManagerStatus manage(const State& state)
    {
        return this->underlying().manage_trajectory(state);
    }
};
} // namespace golem

#endif /* GOLEM_TRAJECTORY_MANAGER_H */
