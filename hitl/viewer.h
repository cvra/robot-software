#pragma once

#include "PhysicsRobot.h"
#include "PhysicsCup.h"
#include "OpponentRobot.h"
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
    int texture_id;

public:
    explicit TableRenderer(int texture_id)
        : texture_id(texture_id)
    {
    }
    void render() override;
    virtual ~TableRenderer() = default;
};

class OpponentRenderer : public Renderable {
    OpponentRobot& opponent;

public:
    OpponentRenderer(OpponentRobot& opponent)
        : opponent(opponent)
    {
    }

    void render() override;
    virtual ~OpponentRenderer() = default;
};

void viewer_init(int argc, char** argv);
void startRendering(std::vector<Renderable*>* r);
