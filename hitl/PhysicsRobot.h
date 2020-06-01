#pragma once

#include <box2d/box2d.h>

class PhysicsRobot {
protected:
    float size_y;
    float pulse_per_mm;
    float pos_left, pos_right;
    b2Body* robotBody;

public:
    PhysicsRobot(b2World& world, float size_x, float size_y, float mass, float pulse_per_mm);
    void ApplyWheelbaseForces(float left, float right);
    void AccumulateWheelEncoders(float dt);
    void GetWheelEncoders(int& left, int& right) const;

    b2Vec2 GetPosition() const
    {
        return robotBody->GetPosition();
    }

    b2Vec2 GetLinearVelocity() const
    {
        return robotBody->GetLinearVelocity();
    }

    float GetAngle() const
    {
        return robotBody->GetAngle();
    }
};

