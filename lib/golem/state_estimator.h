#ifndef GOLEM_STATE_ESTIMATOR_H
#define GOLEM_STATE_ESTIMATOR_H

#include "crtp.h"

namespace golem {
// Models the state estimator of your system
// Accepts any class that implements the following functions for given State and Feedback types:
// - void update_state(const Feedback&) const;
// - State get_state() const;
template <typename StateEstimatorImpl, typename State, typename Feedback>
class StateEstimator : public Crtp<StateEstimatorImpl, StateEstimator> {
public:
    State get() const
    {
        return this->underlying().get_state();
    }
    void update(const Feedback& in)
    {
        return this->underlying().update_state(in);
    }
};
} // namespace golem

#endif /* GOLEM_STATE_ESTIMATOR_H */
