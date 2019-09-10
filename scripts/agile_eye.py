from collections import deque
import cv2
import imutils
import numpy as np
from dynamixel_sdk import *
import config
from time import sleep
import os

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

ONE_BYTE_TRANSMITION        = 1
TWO_BYTE_TRANSMITION        = 2
FOUR_BYTE_TRANSMITION       = 4

DXL1_ID                     = 1
DXL2_ID                     = 2
DXL3_ID                     = 3
DXL1_OFFSET                 = 1280
DXL2_OFFSET                 = 1418
DXL3_OFFSET                 = 1550

BAUDRATE                    = 57600

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
POSITION_MODE               = 3
VELOCITY_MODE               = 1
DXL_VELOCITY_LIMIT          = 360
READ_POSITION               = 0
READ_VELOCITY               = 1
READ_CURRENT                = 2

# Control Table Adresses
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_VELOCITY       = 128
ADDR_PRESENT_CURRENT        = 126
ADDR_OPERATING_MODE         = 11
ADDR_VELOCITY_LIMIT         = 44

ERROR_POSITION_THRESHOLD    = 50    

"""A Class made for controlling MX-106 dynamixel servos

lesserKeyOfMohammad is a class that connects to TAARLAB agile eye robot
dynamixel servos, performing basic tasks
Coded by Mohammad Rabiei, during Mechantronics Enginnering final
project.
"""

class AGILE:
    """
    creates a serial connection with USB port, sending control commands
    and receiving state variables from encoders and sensors

    ...

    Attributes
    ----------
    deviceName : str
        device name is address of USB port in os
        default value in Linux based system is '/dev/ttyUSB0'

    warnMe     : str
        sets serial port transmition warnings and errors 'on' and 'off'


    Methods
    -------
    openPort()
        opens serial connection with DXL driver board

    setBaudRate()
        sets baud rate of data transmition

    change_torque_mode(mode)
        turns DXL torque-enable on or off
        when torque-enable is on, servo motor creates torque, keeping
        the shaft steady, while locking some registers, for example
        operation-mode registers.

    home()
        rotates servos so that agile eye gets into its homing position

    wait_for_key()
        creates a delay between robot homing and motion

    initiate()
        switches operating mode to velocity mode

    move_with_velocity(velocity)
        inputs a vector of 3 integers, each correspending to a servo.
        afterwards, each servo moves with desired angular velocity
    """

    def __init__(self, deviceName, warnMe='off'):
        # Private
        self.DEVICENAME                  = deviceName
        self.PROTOCOL_VERSION            = 2.0
        self.portHandler                 = PortHandler(deviceName)
        self.packetHandler               = PacketHandler(self.PROTOCOL_VERSION)
        self.warn                        = warnMe
        self.prePos                      = [0, 0, 0]

    def openPort(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def setBaudRate(self):
        self.portHandler.setPacketTimeout(10)
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def transmit(self, byte, id, addr, value):
        """
        this method uses packetHandler writeTxRx method to transmit
        control signals to dynamixel registers

        using this method, code becomes cleaner

        ...

        Attributes
        ----------
        byte
            number of transmition bytes

        id
            indicates target servo ID

        addr
            address of target register

        value
            transmitted value that will be written in addr register of
            'id'th servo
        """

        if (self.warn=='off'):
            if (byte==1):
                self.packetHandler.write1ByteTxRx(self.portHandler, id, addr, value)
            elif (byte==2):
                self.packetHandler.write2ByteTxRx(self.portHandler, id, addr, value)
            elif (byte==4):
                self.packetHandler.write4ByteTxRx(self.portHandler, id, addr, value)
        elif (self.warn=='on'):
            if (byte == 1):
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, addr, value)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            elif (byte == 2):
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, addr, value)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            elif (byte == 4):
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, addr, value)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def change_torque_mode(self, mode):
        addr = ADDR_TORQUE_ENABLE
        self.transmit(ONE_BYTE_TRANSMITION, DXL1_ID, addr, mode)
        self.transmit(ONE_BYTE_TRANSMITION, DXL2_ID, addr, mode)
        self.transmit(ONE_BYTE_TRANSMITION, DXL3_ID, addr, mode)

    def change_operating_mode(self, mode):
        addr = ADDR_OPERATING_MODE
        self.transmit(ONE_BYTE_TRANSMITION, DXL1_ID, addr, mode)
        self.transmit(ONE_BYTE_TRANSMITION, DXL2_ID, addr, mode)
        self.transmit(ONE_BYTE_TRANSMITION, DXL3_ID, addr, mode)

    def limit_velocity(self, limit):
        addr = ADDR_VELOCITY_LIMIT
        self.transmit(FOUR_BYTE_TRANSMITION, DXL1_ID, addr, limit)
        self.transmit(FOUR_BYTE_TRANSMITION, DXL2_ID, addr, limit)
        self.transmit(FOUR_BYTE_TRANSMITION, DXL3_ID, addr, limit)

    def go_to_position(self, desiredPosition):
        addr = ADDR_GOAL_POSITION
        self.transmit(FOUR_BYTE_TRANSMITION, DXL1_ID, addr, desiredPosition[0])
        self.transmit(FOUR_BYTE_TRANSMITION, DXL2_ID, addr, desiredPosition[1])
        self.transmit(FOUR_BYTE_TRANSMITION, DXL3_ID, addr, desiredPosition[2])

    def go_to_home_position(self):
        self.go_to_position([DXL1_OFFSET, DXL2_OFFSET, DXL3_OFFSET])

    def move_with_velocity(self, velocity):
        # print ( "catched vel, vel0 ; ", velocity, velocity[0])
        addr = ADDR_GOAL_VELOCITY
        self.transmit(FOUR_BYTE_TRANSMITION, DXL1_ID, addr, velocity[0])
        self.transmit(FOUR_BYTE_TRANSMITION, DXL2_ID, addr, velocity[1])
        self.transmit(FOUR_BYTE_TRANSMITION, DXL3_ID, addr, velocity[2])

    def read_from_motor(self, mode):
        if (mode == READ_POSITION):
            addr = ADDR_PRESENT_POSITION
        elif (mode == READ_VELOCITY):
            addr = ADDR_PRESENT_VELOCITY
        elif (mode == READ_CURRENT):
            addr = ADDR_PRESENT_CURRENT

        m1 = self.packetHandler.read4ByteTxRx(self.portHandler, DXL1_ID, addr)
        m2 = self.packetHandler.read4ByteTxRx(self.portHandler, DXL2_ID, addr)
        m3 = self.packetHandler.read4ByteTxRx(self.portHandler, DXL3_ID, addr)

        newPos = [0.0726*(m1[0]-DXL1_OFFSET), 0.0726*(m2[0]-DXL2_OFFSET), 0.0726*(m3[0]-DXL3_OFFSET)]
        if(np.abs(np.max(np.abs(np.array(newPos)) - np.abs(np.array(self.prePos))) < ERROR_POSITION_THRESHOLD)):
            self.prePos = newPos
            return newPos
        else:
            print(newPos, self.prePos)
            return self.prePos

    def home(self):
        """
        moves robot to homing position

        first, releases servo, by setting TORQUE_MODE to TORQUE_DISABLE
        then changes OPERATING_MODE to POSITION_MODE, limits maximum
        velocity of servos, enables TORQUE_ENABLE and goes to homing
        position, then waits for 1sec to ensure end-effector stabillity.
        """

        self.change_torque_mode(TORQUE_DISABLE)
        self.change_operating_mode(POSITION_MODE)
        self.limit_velocity(DXL_VELOCITY_LIMIT)
        self.change_torque_mode(TORQUE_ENABLE)
        self.go_to_home_position()
        sleep(1)

    def initiate(self):
        """
            change OPERATING_MODE to VELOCITY_MODE

            after homing process is completed, rest of robot control
            is done by velocity mode, because it ensures smoother motion
            with less chattering.
        """

        self.change_torque_mode(TORQUE_DISABLE)
        self.change_operating_mode(VELOCITY_MODE)
        self.change_torque_mode(TORQUE_ENABLE)

    def wait_for_key(self):
        """
            creates a delay between homing and motion processes.

            delay ends with user entering any key.
        """

        print("Homing completed successfully")
        print("Press any key to continue!")
        getch()

    def jacob(self, jointTheta, endTheta):
        t1 = jointTheta[0]
        t2 = jointTheta[1]
        t3 = jointTheta[2]
        psi = endTheta[0]
        theta = endTheta[1]
        phi = endTheta[2]

        w1 = np.mat([0, -np.cos(t1), -np.sin(t1)])
        w2 = np.mat([-np.sin(t2), 0, -np.cos(t2)])
        w3 = np.mat([-np.cos(t3), -np.sin(t3), 0])

        v3 = np.mat([
            +np.sin(phi)*np.cos(psi) - np.cos(phi)*np.sin(theta)*np.sin(psi),
            -np.cos(phi)*np.cos(psi) - np.sin(phi)*np.sin(theta)*np.sin(psi),
            -np.cos(theta)*np.sin(psi)
        ])
        v2 = np.mat([
            -np.cos(phi)*np.cos(theta),
            -np.sin(phi)*np.cos(theta),
            +np.sin(theta)
        ])
        v1 = np.mat([
            -np.sin(phi)*np.sin(psi) - np.cos(phi)*np.sin(theta)*np.cos(psi),
            +np.cos(phi)*np.sin(psi) - np.sin(phi)*np.sin(theta)*np.cos(psi),
            -np.cos(theta)*np.cos(psi)
        ])

        forwardJac = np.mat(np.concatenate([[np.cross(w1, v1)[0]], [np.cross(w2, v2)[0]], [np.cross(w3, v3)[0]]], axis=0))
        inverseJac = np.mat([[forwardJac[0, 0], 0, 0],
            [0, forwardJac[1, 1], 0],
            [0, 0, forwardJac[2, 2]]])

        return inverseJac.I * forwardJac

class EYE:
    def __init__(self, tailLength=30):
        self.cam = [];
        self.threshLow  = (20, 60, 175)
        self.threshHigh = (20, 255, 230)
        self.tailLength = tailLength
        self.tail = deque(maxlen=tailLength)
        self.frame = []
        self.ret = []

    def wake_up(self):
        self.cam = cv2.VideoCapture(1)

    def see(self):
        self.ret, frame = self.cam.read()
        self.frame = imutils.resize(frame, width=600)

    def did_see(self):
        return self.ret

    def infer(self):
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.threshLow, self.threshHigh)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 10:
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
    				(0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)
        self.tail.appendleft(center)
        for i in range(1, len(self.tail)):
            if self.tail[i - 1] is None or self.tail[i] is None:
                continue
            thickness = int(np.sqrt(self.tailLength / float(i + 1)) * 2.5)
            cv2.line(self.frame, self.tail[i - 1], self.tail[i], (0, 0, 255), thickness)

    def show(self):
        cv2.imshow("Frame", self.frame)

    def sleep(self):
        self.cam.release()
        cv2.destroyAllWindows()
