#include <iostream>
#include <fstream>
#include <absl/synchronization/mutex.h>
#include <cvra/motor/control/Voltage.hpp>
#include <cvra/odometry/WheelEncoder.hpp>
#include <box2d/box2d.h>
#include <uavcan_linux/uavcan_linux.hpp>
#include <thread>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "motor_board_emulator.h"
#include "wheel_encoders_emulator.h"
#include "sensor_board_emulator.h"
#include "actuator_board_emulator.h"
#include <error/error.h>
#include "logging.h"
#include "viewer.h"
#include "PhysicsRobot.h"
#include "PhysicsCup.h"
#include "png_loader.h"

ABSL_FLAG(int, first_uavcan_id, 42, "UAVCAN ID of the first board."
                                    " Subsequent ones will be incremented by 1 each.");
ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to connect the emulation to");
ABSL_FLAG(std::string, position_log, "robot_pos.txt", "File in which to write the position log.");
ABSL_FLAG(std::string, table_texture, "hitl/table.png", "File to use as table texture (PNG format).");
ABSL_FLAG(bool, enable_gui, true, "Enables or not the graphical view.");

float clamp(float min, float val, float max)
{
    if (val > max) {
        return val;
    }
    if (val < min) {
        return min;
    }
    return val;
}

void makeBorders(b2World& world)
{
    // horizontal borders
    float size_x = 3.f, size_y = 2.f;
    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(0.5 * size_x, -0.1);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(0.5 * size_x, 0.1);

        borderBody->CreateFixture(&box, 1.);
    }

    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(0.5 * size_x, size_y + 0.1);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(0.5 * size_x, 0.1);

        borderBody->CreateFixture(&box, 1.);
    }

    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(-0.1, 0.5 * size_y);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(0.1, 0.5 * size_y);

        borderBody->CreateFixture(&box, 1.);
    }

    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(size_x + 0.1, 0.5 * size_y);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(0.1, 0.5 * size_y);

        borderBody->CreateFixture(&box, 1.);
    }

    // Borders on the table
    const float border_width = 0.022;
    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(0.9, 1.925);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(border_width * 0.5, 0.075);

        borderBody->CreateFixture(&box, 1.);
    }
    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(2.1, 1.925);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(border_width * 0.5, 0.075);

        borderBody->CreateFixture(&box, 1.);
    }
    {
        b2BodyDef def;
        def.type = b2_staticBody;
        def.position.Set(1.5, 1850);

        b2Body* borderBody = world.CreateBody(&def);

        b2PolygonShape box;
        box.SetAsBox(border_width * 0.5, 0.150);

        borderBody->CreateFixture(&box, 1.);
    }
}

std::vector<CupRenderer> create_cups(b2World& world)
{
    struct cup_init_s {
        CupColor color;
        b2Vec2 pos;
    };

    std::vector<cup_init_s> cup_init = {
        {CupColor::GREEN, {0.3, 0.4}},
        {CupColor::RED, {0.3, 1.2}},
        {CupColor::RED, {0.45, 0.51}},
        {CupColor::GREEN, {0.45, 1.08}},
        {CupColor::GREEN, {0.67, 0.1}},
        {CupColor::RED, {0.95, 0.4}},
        {CupColor::GREEN, {1.1, 0.8}},
        {CupColor::RED, {1.27, 1.2}},

        {CupColor::RED, {1.065, 1.65}},
        {CupColor::GREEN, {1.335, 1.65}},
        {CupColor::GREEN, {1.005, 1.955}},
        {CupColor::RED, {1.395, 1.955}}};

    auto symmetry = [](cup_init_s init) {
        cup_init_s res;
        if (init.color == CupColor::RED) {
            res.color = CupColor::GREEN;
        } else {
            res.color = CupColor::RED;
        }

        res.pos.x = 3. - init.pos.x;
        res.pos.y = init.pos.y;
        return res;
    };

    int initial_size = cup_init.size();
    for (int i = 0; i < initial_size; i++) {
        cup_init.push_back(symmetry(cup_init[i]));
    }

    std::vector<CupRenderer> renderers;

    for (auto init : cup_init) {
        auto cup = std::make_shared<PhysicsCup>(world, init.pos);
        renderers.emplace_back(cup, init.color);
    }

    return renderers;
}

int main(int argc, char** argv)
{
    absl::SetProgramUsageMessage("Emulates one of CVRA "
                                 "motor control board over UAVCAN");
    absl::ParseCommandLine(argc, argv);

    std::ofstream position_log;
    position_log.open(absl::GetFlag(FLAGS_position_log), std::ios::trunc);

    b2Vec2 gravity(0.0f, 0.0f);
    b2World world(gravity);

    makeBorders(world);

    // TODO: Smoother configuration
    auto size = 0.212f;
    auto mass = 4.;
    b2Vec2 initial_pos(0.2, 1.0);
    float initial_heading = 0.;
    float pulse_per_mm = 162;
    PhysicsRobot robot(world, size, size, mass, pulse_per_mm, initial_pos, initial_heading);

    logging_init();

    int board_id = absl::GetFlag(FLAGS_first_uavcan_id);
    std::string iface = absl::GetFlag(FLAGS_can_iface);

    UavcanMotorEmulator left_motor(iface, "left-wheel", board_id++);
    UavcanMotorEmulator right_motor(iface, "right-wheel", board_id++);
    WheelEncoderEmulator wheels(iface, "encoders", board_id++);
    SensorBoardEmulator sensor(iface, "front-left-sensor", board_id++);
    sensor.set_distance(42);

    ActuatorBoardEmulator actuator(iface, "actuator-front-center", board_id++);

    float pressure[2] = {50e3, 80e3};
    actuator.set_pressure(pressure);

    right_motor.start();
    left_motor.start();
    wheels.start();
    sensor.start();
    actuator.start();

    auto cups = create_cups(world);

    std::thread world_update([&]() {
        while (true) {
            const float dt = 0.01;
            std::this_thread::sleep_for(dt * std::chrono::seconds(1));

            const float f_max = 8.;
            robot.ApplyWheelbaseForces(
                clamp(-f_max, -left_motor.get_voltage(), f_max),
                clamp(-f_max, right_motor.get_voltage(), f_max));

            // number of iterations taken from box2d manual
            const int velocityIterations = 10;
            const int posIterations = 30;

            world.Step(dt, velocityIterations, posIterations);

            robot.AccumulateWheelEncoders(dt);

            int left, right;
            robot.GetWheelEncoders(left, right);
            wheels.set_encoders(-left, right);

            auto pos = robot.GetPosition();
            auto vel = robot.GetLinearVelocity();

            NOTICE_EVERY_N(10, "pos: %.3f %.3f", pos.x, pos.y);
            position_log << pos.x << ",";
            position_log << pos.y << ",";
            position_log << vel.x << ",";
            position_log << vel.y << std::endl;
        }
    });

    if (absl::GetFlag(FLAGS_enable_gui)) {
        viewer_init(argc, argv);

        int width, height;
        int texture_id = texture_load(absl::GetFlag(FLAGS_table_texture).c_str(), &width, &height);

        NOTICE("%d, %d", width, height);

        if (texture_id == 0) {
            ERROR("Could not load table texture");
        }

        TableRenderer table_renderer(texture_id);
        RobotRenderer robot_renderer(robot);
        std::vector<Renderable*> renderables{&table_renderer, &robot_renderer};

        for (auto& cup : cups) {
            renderables.push_back(&cup);
        }

        startRendering(&renderables);
    }

    while (true) {
    }

    return 0;
}
