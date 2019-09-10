import os
import imutils
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from imutils.video import VideoStream
from collections import deque
from dynamixel_sdk import *
import cv2

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import config
from agile_eye import AGILE
# from agile_eye import EYE

TORQUE_DISABLE              = 0
READ_POSITION               = 0
READ_VELOCITY               = 1

def wavemake(ampl, freq, phas, time, ticks):
    a1 = ampl[0]; a2 = ampl[1]; a3 = ampl[2]
    f1 = freq[0]; f2 = freq[1]; f3 = freq[2]
    p1 = phas[0]; p2 = phas[1]; p3 = phas[2]
    t = np.linspace(0, time, ticks)
    v1 = a1 * f1 * np.cos(f1 * t - p1)
    v2 = a2 * f2 * np.cos(f2 * t - p2)
    v3 = a3 * f3 * np.cos(f3 * t - p3)
    return np.int32(np.transpose([v1, v2, v3]))


gazer = AGILE('/dev/ttyUSB0', warnMe='off')
motion = wavemake([0, 20, 0], [4, 1, 9], [0, 0, 0], 10, 150)

gazer.openPort()
gazer.setBaudRate()
gazer.home()
gazer.wait_for_key()
gazer.initiate()

list1, list2 = [], []
for item in motion:
    gazer.move_with_velocity(item)
    list1.append(gazer.read_from_motor(READ_POSITION)[1])
    list2.append(item[1])

gazer.home()
print("")
plt.plot(list1)
plt.plot(list2)
plt.show()
