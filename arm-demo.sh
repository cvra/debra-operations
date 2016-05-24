#!/bin/bash

python comm/publish.py /right-arm/setpoint "[0, 0.12, -0.0, 0.18, 1.67]"
sleep 1
python comm/publish.py /right-arm/setpoint "[0, 0.075, -0.0, 0.18, 1.67]"
sleep 0.5
python comm/call.py /actuator/voltage "[right-pump-2, 15]"
python comm/call.py /actuator/voltage "[right-pump-3, 15]"
python comm/publish.py /right-arm/setpoint "[0, 0.075, -0.0, 0.16, 1.67]"
sleep 0.5
python comm/publish.py /right-arm/setpoint "[0, 0.075, -0.0, 0.18, 1.67]"
sleep 0.5
python comm/publish.py /right-arm/setpoint "[0, 0.12, -0.0, 0.18, 1.67]"
sleep 0.5
python comm/publish.py /right-arm/setpoint "[0, 0.12, -0.12, 0.18, 1.67]"
sleep 1
python comm/publish.py /right-arm/setpoint "[2, 0, -0.25, 0.18, 0]"
sleep 2
python comm/publish.py /right-arm/setpoint "[2, 0, -0.25, 0.06, 0]"
sleep 2
python comm/call.py /actuator/voltage "[right-pump-2, 0]"
python comm/publish.py /right-arm/setpoint "[2, 0, -0.25, 0.125, 0]"
sleep 1
python comm/publish.py /right-arm/setpoint "[3, 0, -0.25, 0.125, 0]"
sleep 1
python comm/publish.py /right-arm/setpoint "[3, 0, -0.25, 0.119, 0]"
sleep 0.3
python comm/call.py /actuator/voltage "[right-pump-3, 0]"
sleep 0.3
python comm/publish.py /right-arm/setpoint "[3, 0, -0.25, 0.18, 0]"
sleep 0.5
python comm/publish.py /right-arm/setpoint "[0, 0.12, -0.0, 0.18, 1.67]"
