from math import copysign
import time
import threading

import zmqmsgbus



TOLERANCE = 0.1
SETPOINT_FACTOR = 2

class Indexer:
    actuators = []

    def __init__(self):
        self.bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                                 pub_addr='ipc://ipc/sink')
        self.node = zmqmsgbus.Node(self.bus)

    class Actuator:
        def __init__(self, string_id):
            self.string_id = string_id
            self.got_first = False
            self.index_pos = 0
            self.pos_setpoint = 0
            self.pos_setpoint_set = False
            self.pos_setpoint_factor = 0.3
            self.stop = False


    def add(self, actuator_ids):
        for actuator_id in actuator_ids:
            new_actuator = self.Actuator(actuator_id)
            self.actuators.append(new_actuator)
            self.turn_on_feedback_stream(new_actuator)

    def turn_on_feedback_stream(self, actuator):
        time.sleep(1)
        self.node.call('/actuator/config_update',
                [{'actuator': {actuator.string_id: {
                    'stream': {
                        'index':        10,
                        'motor_pos':    30
                    }
                }}}])

    def turn_off_feedback_stream(self, actuator):
        self.node.call('/actuator/config_update',
                [{'actuator': {actuator.string_id: {
                    'stream': {
                        'index':        0,
                        'motor_pos':    0
                    }
                }}}])

    def index_callback(self, actuator, index_pos):
        if not actuator.stop:
            if not actuator.got_first:
                actuator.index_pos = index_pos
                actuator.got_first = True
                actuator.pos_setpoint = actuator.index_pos \
                    + copysign(0.3, actuator.pos_setpoint - actuator.index_pos)
                self.node.call('/actuator/position',
                        [actuator.string_id, actuator.pos_setpoint])
            else:
                if abs(index_pos - actuator.index_pos) >= TOLERANCE:
                    actuator.stop = True
                    self.turn_off_feedback_stream(actuator)
                    actuator.index_pos = (actuator.index_pos + index_pos) / 2
                    self.node.call('/actuator/position',
                            [actuator.string_id, actuator.index_pos])


    def pos_callback(self, actuator, position):
        if not actuator.stop:
            if not actuator.pos_setpoint_set:
                actuator.pos_setpoint = position + actuator.pos_setpoint_factor
                self.node.call('/actuator/position',
                        [actuator.string_id, actuator.pos_setpoint])
                actuator.pos_setpoint_set = True
                actuator.pos_setpoint_factor *= -SETPOINT_FACTOR
            else:
                if abs(actuator.pos_setpoint - position) <= TOLERANCE:
                    actuator.pos_setpoint = position + actuator.pos_setpoint_factor
                    self.node.call('/actuator/position',
                            [actuator.string_id, actuator.pos_setpoint])
                    actuator.pos_setpoint_factor *= -SETPOINT_FACTOR

    def message_cb(self, topic, msg):
        topic = topic.split('/')
        string_id = topic[2]
        sub_topic = topic[3]

        for actuator in self.actuators:
            if actuator.string_id == string_id:
                if sub_topic == 'index':
                    self.index_callback(actuator, msg)
                elif sub_topic == 'position':
                    self.pos_callback(actuator, msg)

    def server_thread(self, server):
        server.serve_forever()
        return

    def start(self):
        self.node.register_message_handler('/actuator/', self.message_cb)

        t = threading.Thread(target=self.server_thread, args=(server,))
        t.start()

        not_done = True

        while not_done:
            not_done = False
            for a in self.actuators:
                if not a.stop:
                    not_done = True
            time.sleep(0.1)

        server.shutdown()

        return {a.string_id: a.index_pos for a in self.actuators}
