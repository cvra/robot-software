#include <math.h>

#include "../golem.h"

struct Position2D {
    float x{};
    float y{};
};

namespace pendulum {
const float GRAVITY = 9.81f;
const float MASS = 1.f;
const float LENGTH = 42.f;
const float FRICTION = 0.1f;

struct System : public golem::System<System, float, float> {
    float frequency;
    float angle;
    float velocity;

    System(float refresh_frequency, float starting_angle, float starting_velocity)
        : frequency(refresh_frequency)
        , angle(starting_angle)
        , velocity(starting_velocity)
    {
    }

    float measure_feedback() const
    {
        return angle;
    }
    void apply_input(const float& torque)
    {
        const float inertia = MASS * LENGTH * LENGTH;
        const float acceleration = torque / inertia - GRAVITY * sinf(angle) / LENGTH - FRICTION * velocity;
        velocity += acceleration / frequency;
        angle += velocity / frequency;
    }
};

struct Kinematics : public golem::StateEstimator<Kinematics, Position2D, float> {
    Position2D pos;

    Kinematics()
    {
        update(0);
    }

    Position2D get_state() const
    {
        return pos;
    }
    void update_state(const float& angle)
    {
        pos.x = LENGTH * cosf(angle);
        pos.y = LENGTH * sinf(angle);
    }
};

struct ProportionalController : public golem::Controller<ProportionalController, float, float, float> {
    float kp;

    float target{0};
    float measured{0};

    ProportionalController(float p)
        : kp(p)
    {
    }

    void set_consign(const float& t)
    {
        target = t;
    }
    float compute_input(const float& state)
    {
        measured = state;
        return kp * error();
    }
    float control_error() const
    {
        return target - measured;
    }
};

struct TrajectoryManager : public golem::TrajectoryManager<TrajectoryManager, float, float> {
    float target_margin = 0.f; // how far we allow the real state to be from the target to announce the target was reached.

    TrajectoryManager(float margin)
        : target_margin(margin)
    {
    }

    float set_target(const float& target)
    {
        return target;
    }
    golem::TrajectoryManagerStatus manage_trajectory(const float& state)
    {
        float error = fabsf(current_target - state);
        return (error > target_margin) ? golem::TrajectoryManagerStatus::Moving : golem::TrajectoryManagerStatus::Ready;
    }
};
} // namespace pendulum
