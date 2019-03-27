#include <math.h>

struct Position2D {
    float x{};
    float y{};
};

namespace pendulum {
const float GRAVITY = 9.81f;
const float MASS = 1.f;
const float LENGTH = 42.f;
const float FRICTION = 0.1f;

template <size_t frequency>
struct System {
    float angle;
    float velocity;

    System(float starting_angle, float starting_velocity)
        : angle(starting_angle)
        , velocity(starting_velocity)
    {
    }

    float measure() const
    {
        return angle;
    }
    void apply(const float& torque)
    {
        const float inertia = MASS * LENGTH * LENGTH;
        const float acceleration = torque / inertia - GRAVITY * sinf(angle) / LENGTH - FRICTION * velocity;
        velocity += acceleration / frequency;
        angle += velocity / frequency;
    }
};

struct Kinematics {
    Position2D pos;

    Kinematics()
    {
        update(0);
    }

    Position2D get() const
    {
        return pos;
    }
    void update(const float& angle)
    {
        pos.x = LENGTH * cosf(angle);
        pos.y = LENGTH * sinf(angle);
    }
};

struct ProportionalController {
    float kp;

    float target{0};
    float measured{0};

    ProportionalController(float p)
        : kp(p)
    {
    }

    void set(const float& t)
    {
        target = t;
    }
    float update(const float& state)
    {
        measured = state;
        return kp * error();
    }
    float error() const
    {
        return target - measured;
    }
};
} // namespace pendulum
