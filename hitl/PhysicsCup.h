#pragma once

#include <box2d/box2d.h>

class PhysicsCup {
protected:
    b2Body* body;

public:
    static constexpr float radius = 0.036;
    PhysicsCup(b2World& world, b2Vec2 pos);
    b2Vec2 GetPosition() { return body->GetWorldCenter(); }
};
