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
#include <error/error.h>
#include "logging.h"
#include "viewer.h"
#include "PhysicsRobot.h"

ABSL_FLAG(int, first_uavcan_id, 42, "UAVCAN ID of the first board."
                                    " Subsequent ones will be incremented by 1 each.");
ABSL_FLAG(std::string, can_iface, "vcan0", "SocketCAN interface to connect the emulation to");
ABSL_FLAG(std::string, position_log, "robot_pos.txt", "File in which to write the position log.");

float clamp(float min, float val, float max)
{
    if (val > max)
        return val;
    if (val < min)
        return min;
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
    b2Vec2 initial_pos(0.6, 0.6);
    float initial_heading = 0.1;
    float pulse_per_mm = 162;
    PhysicsRobot robot(world, size, size, mass, pulse_per_mm, initial_pos, initial_heading);

    logging_init();

    int board_id = absl::GetFlag(FLAGS_first_uavcan_id);
    std::string iface = absl::GetFlag(FLAGS_can_iface);

    UavcanMotorEmulator left_motor(iface, "left-wheel", board_id++);
    UavcanMotorEmulator right_motor(iface, "right-wheel", board_id++);
    WheelEncoderEmulator wheels(iface, "encoders", board_id++);

    right_motor.start();
    left_motor.start();
    wheels.start();

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

    startRendering(argc, argv, robot);

    return 0;
}
