This repository contains a modified (experimental) DCC++ Base Station source code.

Original DCC++ source code taken from https://github.com/DccPlusPlus/BaseStation/tree/master-2.0
All rights belong to the original author, except the changes/additions I have made.

Changes to include support for the Arduino Due board (84 MHz ARM, 3.3V board voltage):

- platform-specific timer setup and interrupt routines
- experimental cutout generation within the interrupt macro function*)
- support for external SPI FRAM as one possible memory option for the Due board (no onboard EEPROM here), default settings: 8 KBytes FRAM, SPI @ 20 MHz, CS pin 52

*) introduces a cutout in the DCC main track signal (preamble section). This requires that the brake traces of the Arduino (DeekRobot here) motor shield are intact.

Attention: Arduino Due support is a first experimental approach. Any warranty / liability excluded.
Do not use for your productive setup. Bugs guaranteed ;-)
