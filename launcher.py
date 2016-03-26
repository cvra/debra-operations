import subprocess
import json
import argparse
import threading
import time

"""
json entries for each command:

"command":
    command with arguments (in list form)

"restart":
    (bool) restart the command if it exits

"depends":
    list of dependencies that must be running before the command is run
    the command is restarted if any of the dependencies restart

"delay":
    delay in seconds after which the command is run if all dependencies
    are satisfied

"restart_delay":
    additional delay after the restart (defaults to 1s)

dependency forces "restart":
    (bool) restart the command when a dependenciy
    restarts, even if it already exited
"""

parser = argparse.ArgumentParser("CVRA process launcher.")
parser.add_argument("config", default='launcher.json', nargs='?',
                    help="json file containing launch config.")
args = parser.parse_args()

services = json.load(open(args.config))
# print(services)

services_running = {name: False for name in services}
service_should_restart = {name: False for name in services}

def wait_for_dependency(dependencies, delay):
    while True:
        while False in [services_running[s] for s in dependencies]:
            time.sleep(0.1)
        time.sleep(delay)
        if False not in [services_running[s] for s in dependencies]:
            return


def restart_dependencies(name):
    dep_list = [s for s in services if name in services[s].get('depends', [])]
    for dep in dep_list:
        if services_running[dep] or services[dep].get('dependency forces restart', False):
            print('> restarting "{}" because of restart of "{}"'.format(dep, name))
            service_should_restart[dep] = True


def start_service(name, service):
    while True:
        restart = service.get('restart', False)

        dependencies = service.get('depends', [])
        delay = service.get('delay', 0)

        wait_for_dependency(dependencies, delay)

        print("> starting", name)

        service_should_restart[name] = False

        p = subprocess.Popen(service['command'])
        services_running[name] = True

        while p.poll() is None:
            time.sleep(0.1)
            if service_should_restart[name]:
                p.terminate()
                restart = True

        services_running[name] = False

        ret = p.returncode
        if ret is not 0:
            print('> ERROR: service "{}" exited with status {}'.format(name, ret))

        restart_dependencies(name)

        time.sleep(service.get('restart_delay', 1))

        while restart is False and service_should_restart[name] is False:
            time.sleep(0.1)


for name in services:
    t = threading.Thread(target=start_service, name=name, args=(name, services[name]))
    t.start()
