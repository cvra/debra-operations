#!/usr/bin/env python3
import zmq
import msgpack
import yaml
import argparse
import logging
import os.path
import time


def zeromq_call(target, service, arg):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(target)
    socket.send(msgpack.packb([service, arg]))
    res = msgpack.unpackb(socket.recv(), encoding='utf8')
    if res[0] == 'ok':
        return res[1]
    else:
        raise Exception('call failed')

def keys_to_str(to_convert):
    """
    Replaces all non str keys by converting them.
    Useful because python yaml takes numerical keys as integers
    """
    if not isinstance(to_convert, dict):
        return to_convert

    return {str(k): keys_to_str(v) for k, v in to_convert.items()}


def create_actuator(target, name):
    """
    Creates the actuator to receive the config.
    """
    return zeromq_call(target, 'actuator_create_driver', name)

def config_split(config):
    """
    Splits a config dict into smaller chunks.
    This helps to avoid sending big config files.
    """
    split = []
    if "actuator" in config:
        for name in config["actuator"]:
            split.append({"actuator": {name: config["actuator"][name]}})
        del(config["actuator"])

    split.append(config)
    return split

def send_config_file(destination, config_file):
    config = yaml.load(config_file)
    config = keys_to_str(config)
    if "actuator" in config:
        for name in config["actuator"].keys():
            print("Creating actuator {}".format(name))
            create_actuator(destination, name)

    for config in config_split(config):
        errors = zeromq_call(destination, 'config_update', config)
        for key, error in errors:
            logging.warning("Error for key '{}': {}".format(key, error))


def main():
    parser = argparse.ArgumentParser("Sends the robot config to the master board.")
    parser.add_argument("config", help="YAML file containing robot config.")
    parser.add_argument("host", help="ZeroMQ address of the bus master node")
    parser.add_argument("-w", "--watch",
                        help="Watch config file for changes.",
                        action="store_true")
    args = parser.parse_args()

    send_config_file(args.host, open(args.config))

    if args.watch:
        print("> watching for file changes...")
        old_mtime = os.path.getmtime(args.config)
        while True:
            try:
                try:
                    mtime = os.path.getmtime(args.config)
                # Some editors delete the file before writing to it
                except FileNotFoundError:
                    pass

                if mtime != old_mtime:
                    old_mtime = mtime
                    send_config_file(args.host, open(args.config))

                time.sleep(0.1)
            except KeyboardInterrupt:
                break
            except:
                logging.exception("Unexpected error occured.")

if __name__ == "__main__":
    main()
