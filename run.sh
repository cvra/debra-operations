#!/bin/bash
source ./env/bin/activate
mkdir -p ipc
sudo env/bin/bootloader_run_app -p /dev/ttyACM0 -a
sleep 5
python launcher.py launcher.json
