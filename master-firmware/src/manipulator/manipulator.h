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
    manipulator::Angles target_tolerance, target_near_window;
    void* mutex;

    pathfinding::Node<Point> nodes[MANIPULATOR_COUNT] = {
        /* MANIPULATOR_INIT */ {{0, 0, 0}},
        /* MANIPULATOR_RETRACT */ {{-0.4, 2.35, 0}},
        /* MANIPULATOR_DEPLOY */ {{1.2221, 0.2866, 1.6251}},
        /* MANIPULATOR_DEPLOY_FULLY */ {{1.2705, 1.5002, 1.4614}},
        /* MANIPULATOR_LIFT_HORZ */ {{1.3344, 1.1287, 0.0}},
        /* MANIPULATOR_PICK_HORZ */ {{0.9956, 0.5278, 0.0}},
        /* MANIPULATOR_PICK_VERT */ {{0.88701, 1.24429, 1.57079}},
        /* MANIPULATOR_LIFT_VERT */ {{1.4116, 0.0572, 2.2754}},
        /* MANIPULATOR_PICK_GOLDONIUM */ {{0.935233, 2.005595, 1.570796}},
        /* MANIPULATOR_LIFT_GOLDONIUM */ {{0.8, 3, 1.5908}},
        /* MANIPULATOR_SCALE */ {{1.4499, 2.5307, 1.5786}},
        /* MANIPULATOR_PUT_ACCELERATOR*/ {{0.5522, 2.8653, 1.5605}},
        /* MANIPULATOR_PUT_ACCELERATOR_DOWN*/ {{0.2617, 2.4874, 1.6114}},
        /* MANIPUKATOR_STORE_FRONT_SAFE */ {{1.0351, 2.5056, 0.5945}},
        /* MANIPUKATOR_STORE_FRONT_0 */ {{0.55592904, 2.29990897, 0}},
        /* MANIPULATOR_STORE_FRONT_1 */ {{0.44061324, 2.40559601, 0}},
        /* MANIPULATOR_STORE_FRONT_2*/ {{0.15942326, 2.76071548, 0.35511947}},
        /* MANIPULATOR_STORE_FRONT_3*/ {{-0.14048151, 2.77174001, 0.366144}},
        /* MANIPULATOR_STORE_FRONT_HIGH */ {{-0.35403161, 2.01076878, 0}},
        /* MANIPULATOR_STORE_FRONT_HIGH_1 */ {{-0.18604025, 2.38740067, 0}},
        /* MANIPULATOR_STORE_FRONT_LOW */ {{-0.4226222, 1.70403752, 0}},
        /* MANIPULATOR_STORE_BACK_1 */ {{-0.44061324, 2.40559601, 0}},
        /* MANIPULATOR_STORE_BACK_2*/ {{-0.98944966, 1.83134011, 0}},
        /* MANIPULATOR_STORE_BACK_3*/ {{-1.49246682, 1.17753052, 0}},
        /* MANIPULATOR_STORE_BACK_HIGH */ {{-1.29581933, 1.06898106, 0}},
        /* MANIPULATOR_STORE_BACK_LOW */ {{-1.1675452, 0.95911452, 0}},
    };
    manipulator_state_t state;

public:
    manipulator::Gripper gripper;

    Manipulator(const std::array<const char*, 3>& motors,
                const std::array<float, 3>& directions,
                const std::array<const char*, 2>& pumps,
                void* _mutex)
        : sys(motors, directions)
        , estimator({0, 0, 0})
        , ctrl({0, 0, 0})
        , mutex(_mutex)
        , state(MANIPULATOR_INIT)
        , gripper(pumps)
    {
        // from initial position, we can only retract
        nodes[MANIPULATOR_INIT].connect(nodes[MANIPULATOR_RETRACT]);

        pathfinding::connect_bidirectional(nodes[MANIPULATOR_RETRACT], nodes[MANIPULATOR_DEPLOY]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_LIFT_HORZ]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_LIFT_HORZ], nodes[MANIPULATOR_PICK_HORZ]);

        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_DEPLOY_FULLY]);

        //stock and mixed with things..
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_PICK_VERT]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_PICK_VERT], nodes[MANIPULATOR_LIFT_VERT]);
        nodes[MANIPULATOR_LIFT_VERT].connect(nodes[MANIPULATOR_RETRACT]);

        //stock front
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_LIFT_VERT], nodes[MANIPULATOR_STORE_FRONT_SAFE]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_SAFE], nodes[MANIPULATOR_STORE_FRONT_0]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_0], nodes[MANIPULATOR_STORE_FRONT_1]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_1], nodes[MANIPULATOR_STORE_FRONT_2]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_2], nodes[MANIPULATOR_STORE_FRONT_3]);
        //stock picking front
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_3], nodes[MANIPULATOR_STORE_FRONT_HIGH_1]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_HIGH_1], nodes[MANIPULATOR_STORE_FRONT_HIGH]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_HIGH], nodes[MANIPULATOR_STORE_FRONT_LOW]);
        //stock back
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_FRONT_3], nodes[MANIPULATOR_STORE_BACK_1]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_BACK_1], nodes[MANIPULATOR_STORE_BACK_2]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_BACK_2], nodes[MANIPULATOR_STORE_BACK_3]);
        //stock picking front
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_BACK_3], nodes[MANIPULATOR_STORE_BACK_HIGH]);
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_STORE_BACK_HIGH], nodes[MANIPULATOR_STORE_BACK_LOW]);

        //put puck in scale
        nodes[MANIPULATOR_LIFT_VERT].connect(nodes[MANIPULATOR_SCALE]);
        nodes[MANIPULATOR_SCALE].connect(nodes[MANIPULATOR_RETRACT]);

        //put puck in accelerator
        pathfinding::connect_bidirectional(nodes[MANIPULATOR_DEPLOY], nodes[MANIPULATOR_PUT_ACCELERATOR]);
        nodes[MANIPULATOR_LIFT_VERT].connect(nodes[MANIPULATOR_PUT_ACCELERATOR]);
        nodes[MANIPULATOR_PUT_ACCELERATOR].connect(nodes[MANIPULATOR_PUT_ACCELERATOR_DOWN]);
        nodes[MANIPULATOR_PUT_ACCELERATOR_DOWN].connect(nodes[MANIPULATOR_RETRACT]);

        nodes[MANIPULATOR_DEPLOY].connect(nodes[MANIPULATOR_PICK_GOLDONIUM]);
        nodes[MANIPULATOR_PICK_GOLDONIUM].connect(nodes[MANIPULATOR_LIFT_GOLDONIUM]);
        nodes[MANIPULATOR_LIFT_GOLDONIUM].connect(nodes[MANIPULATOR_RETRACT]);
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
    void set_window(const Angles& tolerances)
    {
        LockGuard lock(mutex);
        target_near_window = tolerances;
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
    bool near_target(void) const
    {
        LockGuard lock(mutex);

        const Angles measured = sys.measure();
        const Angles consign = sys.last_raw;

        for (size_t i = 0; i < 3; i++) {
            if (fabsf(measured[i] - consign[i]) > target_near_window[i]) {
                return false;
            }
        }
        return true;
    }
};
} // namespace manipulator

#endif /* MANIPULATOR_MANIPULATOR_H */
