# Presentation

At [CVRA](http://www.cvra.ch/), we take part in the robotics competition [Eurobot](https://www.coupederobotique.fr/).
The rules change every year, and are released in September-October, the national and international competitions take place in April-June of the following year.
Which gives us 6-7 months to build one or two mobile robots to solve the tasks of the year.

Until 2014, the robots we developed were centralized systems.
An FPGA board powered the robot: reading sensors, controlling actuators, and running all the control loops.
An auxiliary computer provided access to higher level features such as computer vision, and it sometimes was used to run the artificial intelligence.

In 2011, we started developing a reusable robot with a differential wheelbase SCARA arms: Debra.
However, the complexity of this robot increased over time.
Increasing the number of motors every year was painful due to additonal software (to configure the FPGA), hardware (motor drive boards), and wiring (all wires go to the body of the robot).
We decided to shift to a distributed architecture.

CAN was selected as main bus over the robot with [UAVCAN](http://uavcan.org/) as protocol.
Each motor would be controlled by a dedicated board that includes a motor drive and a microcontroller, taking care of control, and only exposing a high-level API over CAN.
This is what we set out to build from 2014.
Since then, we have developed many other boards that communicate over UAVCAN.

Today our average robot contains 15-20 boards over CAN:
- [Motor boards](./motor.html) that can control a DC motor in voltage, torque, velocity, or position.
- [IO boards](./io.html) that can interface digital inputs and outputs (including PWM for controlling RC servos).
- [Sensor boards](./sensor.html) that have a time of flight distance sensor and an RGB color sensor onboard.
- [Beacon boards](./beacon.html) that enable centimeter accurate global positioning.

with a Master board to orchestrate everything.

Along these boards, we also developed tools to make our life easier:
- A [CAN USB dongle](./dongle.html) to inspect traffic over the bus from our computers.
- A [CAN bootloader](./bootloader.html) to enable firmware updates over CAN, so we don't need to disassemble our robots to update single modules.
