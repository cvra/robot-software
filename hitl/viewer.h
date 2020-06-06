#pragma once

#include "PhysicsRobot.h"
#include "PhysicsCup.h"
#include <vector>
#include <memory>

class Renderable {
public:
    virtual void render() = 0;
};

enum class CupColor {
    GREEN,
    RED
};

class CupRenderer : public Renderable {
    std::shared_ptr<PhysicsCup> cup;
    CupColor color;

public:
    CupRenderer(std::shared_ptr<PhysicsCup> cup, CupColor color)
        : cup(cup)
        , color(color){};
    void render() override;
    virtual ~CupRenderer() = default;
};

class RobotRenderer : public Renderable {
    PhysicsRobot& robot;

public:
    RobotRenderer(PhysicsRobot& robot)
        : robot(robot)
    {
    }
    void render() override;
    virtual ~RobotRenderer() = default;
};

class TableRenderer : public Renderable {
public:
    void render() override;
};

void startRendering(int argc, char** argv, std::vector<Renderable*>* renderables);
