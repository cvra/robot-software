#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <iostream>
#include "motor_board_emulator.h"
#include "wheel_encoders_emulator.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_replace.h"
#include "absl/types/optional.h"
#include "actuator_board_emulator.h"
#include <string>

using namespace webots;

void update_speed(Motor* m, UavcanMotorEmulator& u, int direction)
{
    constexpr float max_speed = 47.6;
    float speed = u.get_voltage() / 10.f * max_speed;

    // Clamp the speed
    speed = std::min(std::max(speed, -max_speed), max_speed);
    speed *= 0.99;
    speed *= direction;

    m->setPosition(INFINITY);
    m->setVelocity(speed);
}

class ActuatorSimulationAdapter {
    Motor* motor;
    DistanceSensor* sensor;
    std::unique_ptr<ActuatorBoardEmulator> actuator;

public:
    ActuatorSimulationAdapter(Motor* m, DistanceSensor* d, std::unique_ptr<ActuatorBoardEmulator> a)
        : motor(m)
        , sensor(d)
        , actuator(std::move(a))
    {
    }

    void update()
    {
        float pos = actuator->get_servo_pos(0);
        motor->setPosition(pos);
        actuator->set_digital_input(sensor->getValue() > 0.5);
    }
};

absl::optional<ActuatorSimulationAdapter> create_actuator_adapter(Robot& robot, std::string iface, int board_id, std::string name)
{
    Motor* m = robot.getMotor(name);
    if (!m) {
        return {};
    }

    std::string sensor_name = absl::StrReplaceAll(name, {{"actuator", "sensor"}});

    DistanceSensor* d = robot.getDistanceSensor(sensor_name);
    d->enable(100);

    if (!d) {
        return {};
    }

    auto emulator = std::make_unique<ActuatorBoardEmulator>(iface, name, board_id);
    emulator->start();

    return ActuatorSimulationAdapter(m, d, std::move(emulator));
}

int main(int /*argc*/, char** /*argv*/)
{
    Robot robot;

    const int timeStep = (int)robot.getBasicTimeStep();

    const std::string iface = "vcan0";
    int board_id = 42;

    Motor* leftMotor = robot.getMotor("left-wheel-motor");
    Motor* rightMotor = robot.getMotor("right-wheel-motor");

    UavcanMotorEmulator left_motor_board(iface, "left-wheel", board_id++);
    UavcanMotorEmulator right_motor_board(iface, "right-wheel", board_id++);
    WheelEncoderEmulator wheels(iface, "encoders", board_id++);

    std::vector<std::string> actuator_names{
        "actuator-front-left", "actuator-front-right", "actuator-front-center",
        "actuator-back-left", "actuator-back-right", "actuator-back-center"};

    std::vector<ActuatorSimulationAdapter> actuator_adapters;

    left_motor_board.start();
    right_motor_board.start();
    wheels.start();

    if (!leftMotor || !rightMotor) {
        std::cerr << "Could not find the robot's motors" << std::endl;
        return 1;
    }

    for (const auto& name : actuator_names) {
        auto a = create_actuator_adapter(robot, iface, board_id++, name);
        if (!a) {
            std::cerr << "Could not find actuator " << name << std::endl;
            return 1;
        }
        actuator_adapters.push_back(std::move(*a));
    }

    leftMotor->getPositionSensor()->enable(100);
    rightMotor->getPositionSensor()->enable(100);

    while (robot.step(timeStep) != -1) {
        update_speed(leftMotor, left_motor_board, -1);
        update_speed(rightMotor, right_motor_board, 1);

        auto leftpos = -leftMotor->getPositionSensor()->getValue();
        auto rightpos = rightMotor->getPositionSensor()->getValue();

        const float wheel_diam_mm = 42;
        const float pulse_per_mm = 162;

        // Complete rotation of the wheel for 2 * pi radians is pi * diam * pulse_per_mm
        // -> rotation for 1 radian is diam * pulse_per_mm / 2
        const float pulse_per_radian = wheel_diam_mm * pulse_per_mm / 2;

        wheels.set_encoders(leftpos * pulse_per_radian, rightpos * pulse_per_radian);
        for (auto& a : actuator_adapters) {
            a.update();
        }
    };

    return 0;
}
