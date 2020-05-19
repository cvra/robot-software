# Wheel Encoder Firmware

The wheel encoder board counts rotary encoder pulses of right and left wheels of the robot and sends the counts with a fixed frequency over UAVCAN.

## Hardware

For a first prototype a CVRA motor board (STM32F303CCT6) is repurposed.
The timer TIM3 and TIM4 are connected right and left wheel encoders and used in counter mode.
The Encoder A/B input signals connected to PB4 and PB5 for TIM3 and PB6 and PB7 for TIM4.


