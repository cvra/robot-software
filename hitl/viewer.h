#pragma once

#include "PhysicsRobot.h"
#include <vector>

class Renderable {
    public:
    virtual void render() = 0;
};

class RobotRenderer : public Renderable {
    PhysicsRobot &robot;
    public:
        RobotRenderer(PhysicsRobot &robot) : robot(robot) {}
        void render() override;
};

class TableRenderer : public Renderable {
    public:
        void render() override;
};


void startRendering(int argc, char** argv, std::vector<Renderable *> *renderables);
