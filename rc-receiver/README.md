# RC Servo signal capture

## Pinout Nucleo64-F401RE

PA8 TIM1_CH1 CN10_23 (D7)
PA6 TIM3_CH1 CN10_13 (D12)
PB6 TIM4_CH1 CN10_17 (D10)
PA0 TIM5_CH1 CN7_28  (A0)
PA2 TIM9_CH1 CN10_37 (D1)

PC6 UART6_TX CN10_04
UART baudrate: 115200

Modifications:
- place 8MHz quartz (X3) with 2x 20pF capacitors (C33, C34)
- SB50 open
- R35 closed
- R37 closed
