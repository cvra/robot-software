#include "PhysicsCup.h"

PhysicsCup::PhysicsCup(b2World& world, b2Vec2 pos)
{
    b2BodyDef def;
    def.type = b2_dynamicBody;

    def.position = pos;

    body = world.CreateBody(&def);

    b2CircleShape shape;
    shape.m_radius = PhysicsCup::radius;

    const float mass = 0.1;
    const float density = mass / (3.14 * PhysicsCup::radius * PhysicsCup::radius);

    body->CreateFixture(&shape, density);
    body->SetLinearDamping(100);
}
