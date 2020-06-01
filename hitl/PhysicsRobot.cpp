#include "PhysicsRobot.h"

PhysicsRobot::PhysicsRobot(b2World& world, float size_x, float size_y, float mass, float pulse_per_mm)
    : size_y(size_y)
    , pulse_per_mm(pulse_per_mm)
    , pos_left(0.f)
    , pos_right(0.f)
{
    b2BodyDef robotDef;
    robotDef.type = b2_dynamicBody;
    robotBody = world.CreateBody(&robotDef);
    b2PolygonShape robotBox;
    robotBox.SetAsBox(size_x / 2, size_y / 2);
    robotBody->CreateFixture(&robotBox, mass / (size_x * size_y));
}

void PhysicsRobot::ApplyWheelbaseForces(float left, float right)
{
    auto left_wheel_pos = robotBody->GetWorldPoint({0., size_y / 2});
    auto right_wheel_pos = robotBody->GetWorldPoint({0., -size_y / 2});

    auto left_wheel_force = robotBody->GetWorldVector({left, 0.});
    auto right_wheel_force = robotBody->GetWorldVector({right, 0.});

    robotBody->ApplyForce(left_wheel_force, left_wheel_pos, true);
    robotBody->ApplyForce(right_wheel_force, right_wheel_pos, true);

    // Apply the tires sideways friction
    // https://www.iforce2d.net/b2dtut/top-down-car
    b2Vec2 rightNormal = robotBody->GetWorldVector({0, 1});
    b2Vec2 lateralVelocity = b2Dot(rightNormal, robotBody->GetLinearVelocity()) * rightNormal;
    b2Vec2 impulse = robotBody->GetMass() * -lateralVelocity;
    robotBody->ApplyLinearImpulse(impulse, robotBody->GetWorldCenter(), true);
}

void PhysicsRobot::AccumulateWheelEncoders(float dt)
{
    auto world_vel = robotBody->GetLinearVelocity();
    auto local_vel = robotBody->GetLocalVector(world_vel);

    pos_left += local_vel.x * dt;
    pos_right += local_vel.x * dt;

    auto angle_vel = robotBody->GetAngularVelocity();
    pos_left -= 0.5 * size_y * angle_vel * dt;
    pos_right += 0.5 * size_y * angle_vel * dt;
}

void PhysicsRobot::GetWheelEncoders(int& left, int& right) const
{
    left = 1000 * pulse_per_mm * pos_left;
    right = 1000 * pulse_per_mm * pos_right;
}

