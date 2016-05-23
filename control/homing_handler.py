from math import copysign
import time

import zmqmsgbus


MIN_TIME = 1
MIN_VELOCITY_PERCENT = 0.4

class Endstopper:
    actuators = []

    def __init__(self):
        self.bus = zmqmsgbus.Bus(sub_addr='ipc://ipc/source',
                                 pub_addr='ipc://ipc/sink')
        self.node = zmqmsgbus.Node(self.bus)

    class Actuator:
        def __init__(self, string_id, velocity_setp):
            self.string_id = string_id
            self.start_time = 0
            self.zero = 0
            self.velocity = 0
            self.velocity_setp = 0
            self.was_fast = False
            self.stop = False

    def add(self, actuator_ids, velocity_setp):
        for actuator_id in actuator_ids:
            new_actuator = self.Actuator(actuator_id, velocity_setp)
            self.actuators.append(new_actuator)
            self.turn_on_feedback_stream(new_actuator)

    def turn_on_feedback_stream(self, actuator):
        time.sleep(1)
        self.node.call('/actuator/config_update',
                {'actuator': {actuator.string_id: {
                    'stream': {
                        'motor_pos':    30
                    }
                }}})

    def turn_off_feedback_stream(self, actuator):
        self.node.call('/actuator/config_update',
                {'actuator': {actuator.string_id: {
                    'stream': {
                        'motor_pos':    0
                    }
                }}})

    def velocity_callback(self, actuator, velocity):
        actuator.velocity = velocity
        if velocity > MIN_VELOCITY_PERCENT * actuator.velocity_setp:
            actuator.was_fast = True

    def pos_callback(self, actuator, position):
        if actuator.start_time == 0:
            actuator.start_time = time.time()
        if not actuator.stop:
            self.node.call('/actuator/velocity',
                    [actuator.string_id, actuator.velocity_setp])
            if velocity < MIN_VELOCITY_PERCENT * actuator.velocity_setp and \
               (actuator.was_fast or time.time() - actuator.start_time > MIN_TIME):
                actuator.stop = True
                actuator.zero = position
                self.node.call('/actuator/position',
                        [actuator.string_id, actuator.zero])
                self.turn_off_feedback_stream(actuator)

    def message_cb(self, topic, msg):
        topic = topic.split('/')
        string_id = topic[2]
        sub_topic = topic[3]

        for actuator in self.actuators:
            if actuator.string_id == string_id:
                if sub_topic == 'velocity':
                    self.velocity_callback(actuator, msg)
                elif sub_topic == 'position':
                    self.pos_callback(actuator, msg)

    def start(self):
        self.node.register_message_handler('/actuator/', self.message_cb)

        not_done = True

        while not_done:
            not_done = False
            for a in self.actuators:
                if not a.stop:
                    not_done = True
            time.sleep(0.1)

        return {a.string_id: a.zero for a in self.actuators}


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
                {'actuator': {actuator.string_id: {
                    'stream': {
                        'index':        10,
                        'motor_pos':    30
                    }
                }}})

    def turn_off_feedback_stream(self, actuator):
        self.node.call('/actuator/config_update',
                {'actuator': {actuator.string_id: {
                    'stream': {
                        'index':        0,
                        'motor_pos':    0
                    }
                }}})

    def index_callback(self, actuator, index_pos):
        index_pos = self.unwind(index_pos)
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

    @staticmethod
    def unwind(a):
        from math import pi
        if a > pi:
            return a - 2*pi
        elif a < -pi:
            return a + 2*pi
        else:
            return a

    def pos_callback(self, actuator, position):
        position = self.unwind(position)
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

    def start(self):
        self.node.register_message_handler('/actuator/', self.message_cb)

        not_done = True

        while not_done:
            not_done = False
            for a in self.actuators:
                if not a.stop:
                    not_done = True
            time.sleep(0.1)

        return {a.string_id: a.index_pos for a in self.actuators}
