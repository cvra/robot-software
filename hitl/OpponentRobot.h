#pragma once

#include <math.h>
#include <box2d/box2d.h>

class OpponentRobot {
    b2Body* robotBody;

public:
    // The current rules allow a max perimeter of 1300 mm, use it to compute the
    // radius of the opponent.
    static constexpr float radius = 1.3f / (2.f * M_PI);

    OpponentRobot(b2World& world, b2Vec2 initial_pos);

    b2Vec2 GetPosition() const
    {
        return robotBody->GetPosition();
    }

    void SetPosition(b2Vec2 pos)
    {
        robotBody->SetTransform(pos, 0.f);
    }
};
