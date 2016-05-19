#!/bin/sh
. ./env/bin/activate

sudo env/bin/bootloader_flash -p /dev/ttyACM0 -a 0x08003800 --device-class motor-board-v1  -b bus/motor-control-firmware/build/motor-control-firmware.bin 20 21 29 31 32 22 23 24 25 27 28
