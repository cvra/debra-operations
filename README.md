# debra-operations

## deploy to embedded PC

- connect to the robot wifi, the raspi will give you an IP & internet connection
- run `deploy.sh` to copy the source code to the robot and install the python modules
- the code is in `/home/cvra/run/`
- connect to the PC (`mosh cvra@192.168.2.10`) and run `run.sh` to run your code

## Setup and/or start virtual environment

(_This only works in fish shell_)

```
source setup_env.sh
```

For some tools (for example graphing) you need additional packages. In this case instead of the above command use:
```
source setup_dev_env.sh
```

To leave the virtual environment: `deactivate`


### Use PyQT on OSX
```
brew install pyqt --with-python3
virtualenv --python=python3 --system-site-packages env
```

## Start ZeroMQ nodes

First start the message bus:
```
python -m zmqmsgbus.tools.bus
```

Then start the node of the master-board:
```
python comm/bus_master_node.py
```

## Forward Bus Topics

To forward the bus topics running on the robot to your laptop run:

```
python -m zmqmsgbus.tools.forwarder --to tcp://192.168.2.10:13370 --bind_from
```
and
```
python -m zmqmsgbus.tools.forwarder --from tcp://192.168.2.10:13371 --bind_to
```


## Example of single actuator control
You need to have an IP `192.168.3.0/24`.

Send config:
```
python config/config_send.py config/config-left-wrist.yaml ipc://ipc/bus_master_server
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

## Network Setup (without embedded PC)

The master-board is on 192.168.3.20.

If you want to run the software-stack on your PC you should have the IP 192.168.3.1 because the master-board uses this address as ntp time server.
Also make sure your `restrict` config (found under `/private/etc/ntp-restrict.conf` on OS X) does not include `noquery`.

In case the embedded PC is connected, it distributes the IPs with DHCP. It also serves NTP.

