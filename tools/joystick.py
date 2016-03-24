import pygame
from pygame.locals import *
import time
import sys
import zmq
import msgpack

POLL_FREQ = 20 #Hz

# Comm
context = zmq.Context()
msg_pub_sock = context.socket(zmq.PUB)
msg_pub_sock.connect("ipc://ipc/sink")

def publish_msg(topic, msg):
    buf = topic.encode('utf8') + b'\0' + msgpack.packb(msg, use_bin_type=True)
    msg_pub_sock.send(buf)

# Pygame Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("no joystick found")
    sys.exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(joystick.get_name())

# pygame always needs a window for some reason
screen = pygame.display.set_mode((400,300))
pygame.display.set_caption('dummy window')

while 1:
    axis = [joystick.get_axis(i) for i in range(0,4)]
    # print(axis)

    # test controller buttons
    buttons = [joystick.get_button(i) for i in range(0,19)]
    # print(buttons)

    if buttons[3] == 1 or buttons[13] == 1: # press START or O button to quit
        break

    publish_msg('/joystick', [axis, buttons])

    pygame.event.get()

    pygame.display.update()
    time.sleep(1/POLL_FREQ)

pygame.quit()
sys.exit()
