#!/bin/bash

set -e

source ./env/bin/activate
mkdir -p ipc
mkdir -p logs
sudo env/bin/bootloader_run_app -p /dev/ttyACM0 -a
sleep 5

python -m zmqmsgbus.tools.bus --in ipc://ipc/sink tcp://*:13370 --out ipc://ipc/source tcp://*:13371 2>&1 | tee logs/bus &
python comm/bus_master_node.py  2>&1 | tee logs/master &
python comm/actuator_node.py  2>&1 | tee logs/bus &
python state_estimation/position.py  2>&1 | tee logs/position &
python -m sicktim561driver.scan  2>&1 | tee logs/lidar_driver &
python state_estimation/localization-stack/lidar_localization.py state_estimation/localization-stack/config.json --logs  2>&1 | tee logs/lidar_position &
sleep 3
python config/config_send.py -w config/config.yaml &
sleep 3
python config/config_send.py -w config/config-right-arm.yaml &
sleep 3
python config/config_send.py -w config/config-left-arm.yaml &
sleep 3
python config/config_send.py -w config/config-pumps.yaml &
sleep 3
python control/proximity_beacon.py  2>&1 | tee logs/beacon &
python control/arm.py config/arm-offsets.yaml  2>&1 | tee logs/arm &
python control/waypoints/waypoints.py  2>&1 | tee logs/waypoints &
python main.py 2>&1 | tee logs/main &

trap "jobs -p | xargs kill" SIGINT
trap "jobs -p | xargs kill" EXIT
read
