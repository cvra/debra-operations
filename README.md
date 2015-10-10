# debra-operations

## Setup and/or start virtual environment
```
source setup_env.sh
```

To leave the virtual environment: `deactivate`


## Start ZeroMQ nodes

First start the message forwarder:
```
python comm/message_forwarder.py
```

Then start the node of the master-board:
```
python comm/bus_master_node.py
```

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

## Plot message (experimental)

First start the ZeroMQ nodes.

Example: plot the postition and velocity of the left-wrist actuator
```
python tools/message_graph.py position:left-wrist.0 position:left-wrist.1
```
