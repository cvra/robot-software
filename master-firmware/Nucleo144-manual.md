# Nucleo144 manual

## Hardware configuration

* Remove ST-Link (will pull NRST line low if not connected to host)
* Solder X3 oscillator: NX3225GD-8.000M-EXS00A-CG04874 (8MHz, 8pF, 20ppm)
  - SB148 and SB163 OFF
  - SB8 and SB9 soldered
  - C37 and C38 soldered with 4.3 pF capacitors
  - SB112 and SB149 OFF
* JP3 set to E5V
* JP4, JP5, JP6 and JP7 closed

Note: the X3 oscillator can be soldered out from the ST-link-V2