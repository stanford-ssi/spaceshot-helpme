#!/bin/bash

~/.arduino15/packages/arduino/tools/openocd/0.9.0-arduino6-static/bin/openocd -d2 -s "/home/boris/.arduino15/packages/arduino/tools/openocd/0.9.0-arduino6-static/share/openocd/scripts/" -f "/home/boris/.arduino15/packages/arduino/hardware/samd/1.6.19/variants/arduino_zero/openocd_scripts/arduino_zero.cfg" -c "telnet_port disabled; init; halt; at91samd bootloader 0; program {{/home/boris/Repos/SAMD21_Mini_Breakout/Firmware/samd21_sam_ba_sparkfun.bin}} verify reset; shutdown"
