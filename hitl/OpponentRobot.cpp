#include "OpponentRobot.h"

OpponentRobot::OpponentRobot(b2World& world, b2Vec2 initial_pos)
{
    b2BodyDef robotDef;
    robotDef.type = b2_staticBody;

    robotDef.position = initial_pos;
    robotDef.angle = 0.f;

    robotBody = world.CreateBody(&robotDef);
    b2CircleShape shape;
    shape.m_radius = OpponentRobot::radius;

    robotBody->CreateFixture(&shape, 1.f);
}
