#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <iostream>
#include "motor_board_emulator.h"
#include "wheel_encoders_emulator.h"
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

int main(int /*argc*/, char** /*argv*/)
{
    Robot robot;

    const int timeStep = (int)robot.getBasicTimeStep();

    const std::string iface = "vcan0";
    int board_id = 42;

    Motor* leftMotor = robot.getMotor("left wheel motor");
    Motor* rightMotor = robot.getMotor("right wheel motor");
    DistanceSensor* distanceSensor = robot.getDistanceSensor("front ultrasonic sensor");
    LED* led = robot.getLED("front left led");

    UavcanMotorEmulator left_motor_board(iface, "left-wheel", board_id++);
    UavcanMotorEmulator right_motor_board(iface, "right-wheel", board_id++);
    WheelEncoderEmulator wheels(iface, "encoders", board_id++);
    ActuatorBoardEmulator actuator_front_left(iface, "actuator-front-left", board_id++);

    left_motor_board.start();
    right_motor_board.start();
    wheels.start();
    actuator_front_left.start();

    if (!leftMotor || !rightMotor) {
        std::cerr << "Could not find the robot's motors" << std::endl;
        return 1;
    }

    leftMotor->getPositionSensor()->enable(100);
    rightMotor->getPositionSensor()->enable(100);

    if (distanceSensor) {
        std::cerr << "Found distance sensor" << std::endl;
        distanceSensor->enable(100);
        distanceSensor->getValue();
    }

    if (led) {
        std::cerr << "Found LED" << std::endl;
    }

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

        if (distanceSensor) {
            float pressure[2];
            pressure[0] = 0.;
            pressure[1] = distanceSensor->getValue() * 1000;
            actuator_front_left.set_pressure(pressure);
        }

        if (led) {
            led->set(255 * actuator_front_left.get_solenoid(0));
        }
    };

    return 0;
}
