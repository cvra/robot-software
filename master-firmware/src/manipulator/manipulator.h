#ifndef MANIPULATOR_MANIPULATOR_H
#define MANIPULATOR_MANIPULATOR_H

#include "manipulator/controller.h"
#include "manipulator/hw.h"
#include "manipulator/state_estimator.h"
#include "manipulator/gripper.h"
#include "manipulator/path.h"

#include "dijkstra.hpp"

#include <math.h>

namespace manipulator {
struct Point {
    float angles[3];
    Point(float a, float b, float c)
        : angles{a, b, c}
    {
    }
};

template <class LockGuard>
class Manipulator {
public:
    manipulator::System sys;
    manipulator::StateEstimator estimator;
    manipulator::Controller ctrl;
    manipulator::Angles target_tolerance;
    void* mutex;

    pathfinding::Node<Point> nodes[MANIPULATOR_COUNT] = {
        /* MANIPULATOR_INIT */ {{0, 0, 0}},
        /* MANIPULATOR_RETRACT */ {{1.2878, -1.2018, 1.1982}},
        /* MANIPULATOR_DEPLOY */ {{1.2221, 0.2866, 1.6251}},
        /* MANIPULATOR_DEPLOY_FULLY */ {{1.2705, 1.5002, 1.4614}},
        /* MANIPULATOR_LIFT_HORZ */ {{1.3344, 1.1287, 0.0}},
        /* MANIPULATOR_PICK_HORZ */ {{0.9956, 0.5278, 0.0}},
        /* MANIPULATOR_PICK_VERT */ {{0.8870, 1.2443, 1.5708}},
    };
    manipulator_state_t state;

public:
    manipulator::Gripper gripper;

    Manipulator(const std::array<float, 3>& link_lengths, void* _mutex)
        : estimator(link_lengths)
        , ctrl(link_lengths)
        , mutex(_mutex)
        , state(MANIPULATOR_INIT)
    {
        // from initial position, we can only retract
        nodes[MANIPULATOR_INIT].connect(nodes[MANIPULATOR_RETRACT]);

        pathfinding::connect_bidirectional(nodes[MANIPULATOR_RETRACT], nodes[MANIPULATOR_DEPLOY]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_LIFT_HORZ]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_LIFT_HORZ], nodes[MANIPULATOR_PICK_HORZ]);

        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_DEPLOY_FULLY]);

        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_PICK_VERT]);
    }

    void set_lengths(const std::array<float, 3>& link_lengths)
    {
        LockGuard lock(mutex);
        estimator.lengths = link_lengths;
        ctrl.lengths = link_lengths;
    }
    void set_offsets(const Angles& offsets)
    {
        LockGuard lock(mutex);
        sys.offsets = offsets;
    }
    void set_target(const Pose2D& pose)
    {
        LockGuard lock(mutex);
        ctrl.set(pose);
    }
    void set_tolerance(const Angles& tolerances)
    {
        LockGuard lock(mutex);
        target_tolerance = tolerances;
    }

    Pose2D update(void)
    {
        LockGuard lock(mutex);
        estimator.update(sys.measure());
        return estimator.get();
    }
    Angles compute_control(void)
    {
        LockGuard lock(mutex);
        return ctrl.update(estimator.get());
    }
    void apply(const Angles& angles)
    {
        LockGuard lock(mutex);
        return sys.apply(angles);
    }

    Angles angles(void) const
    {
        LockGuard lock(mutex);
        return sys.measure();
    }
    bool reached_target(void) const
    {
        LockGuard lock(mutex);

        const Angles measured = sys.measure();
        const Angles consign = sys.last_raw;

        for (size_t i = 0; i < 3; i++) {
            if (fabsf(measured[i] - consign[i]) > target_tolerance[i]) {
                return false;
            }
        }
        return true;
    }
};
} // namespace manipulator

#endif /* MANIPULATOR_MANIPULATOR_H */
