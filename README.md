# debra-operations

## Setup and/or start virtual environment
```
source setup_env.sh
```

To leave the virtual environment: `deactivate`


## Example of single actuator control
You need to have an IP `192.168.3.0/24`.

Send config:
```
python config/config_send.py config/config-left-wrist.yaml 192.168.3.20
```

Send setpoint:
```
python control/actuator-control-interface/send_setpoint_test.py -a left-wrist -p 192.168.3.20 3.14
```
